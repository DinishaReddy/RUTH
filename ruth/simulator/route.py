import logging
import math
from datetime import datetime, timedelta
from typing import List, Tuple

from .queues import QueuesManager
from ..data.map import Map
from ..data.segment import Segment, SegmentPosition, SpeedMps, SpeedKph, LengthMeters, speed_kph_to_mps
from .simulation import FCDRecord
from ..globalview import GlobalView
from ..vehicle import Vehicle

logger = logging.getLogger(__name__)

turn_log_count = 0


def move_on_segment(
        vehicle: Vehicle,
        driving_route_part: List[Segment],
        current_time: datetime,
        gv_db: GlobalView,
        routing_map: Map,
        los_vehicles_tolerance: timedelta = timedelta(seconds=0)
) -> Tuple[datetime, SegmentPosition, SpeedMps]:
    """
    Moves the car on its current segment.
    Returns (time, position, speed) at the end of the movement.
    """
    segment_position = vehicle.segment_position
    start_position = segment_position.position
    current_segment = driving_route_part[0]
    assert segment_position.position <= current_segment.length

    turn_type = 'straight'
    abs_angle_deg = 0.0
    next_segment = None
    prev_segment = None

    if start_position == current_segment.length:
        # if the vehicle is at the end of a segment and there are more segments in the route
        if vehicle.has_next_segment_closed(routing_map):
            return current_time + vehicle.frequency, vehicle.segment_position, SpeedMps(0.0)
        
        # Save the previous segment before switching
        prev_segment = current_segment
        
        # Calculate turn type before moving to next segment
        next_segment = driving_route_part[1]
        
        if current_segment.node_from == next_segment.node_from and current_segment.node_to == next_segment.node_to:
            logger.warning(f"Vehicle {vehicle.id} moving to same edge, classifying as straight")
            turn_type = 'straight'
            abs_angle_deg = 0.0
        else:
            from_pos = (routing_map.network.nodes[current_segment.node_from]['x'], routing_map.network.nodes[current_segment.node_from]['y'])
            to_pos = (routing_map.network.nodes[current_segment.node_to]['x'], routing_map.network.nodes[current_segment.node_to]['y'])
            next_to_pos = (routing_map.network.nodes[next_segment.node_to]['x'], routing_map.network.nodes[next_segment.node_to]['y'])
            
            current_vector = (to_pos[0] - from_pos[0], to_pos[1] - from_pos[1])
            next_vector = (next_to_pos[0] - to_pos[0], next_to_pos[1] - to_pos[1])
            
            current_angle = math.atan2(current_vector[1], current_vector[0])
            next_angle = math.atan2(next_vector[1], next_vector[0])
            angle_diff = next_angle - current_angle
            
            # Normalize to -pi to pi
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            abs_angle = abs(angle_diff)
            if abs_angle < math.radians(30):
                turn_type = 'straight'
            elif abs_angle > math.radians(150):
                turn_type = 'u_turn'
            else:
                turn_type = 'turn'
            
            abs_angle_deg = abs_angle * 180 / math.pi
        next_segment = driving_route_part[1]
        
        # if the vehicle can move to the next segment, work with the next segment
        start_position = LengthMeters(0.0)
        segment_position = SegmentPosition(segment_position.index + 1, start_position)
        current_segment = next_segment

    if segment_position.index == (len(vehicle.osm_route) - 1) and start_position == current_segment.length:
        # the end of the driving route
        level_of_service = 1.0
    else:
        level_of_service = gv_db.level_of_service_in_front_of_vehicle(current_time, current_segment, vehicle.id,
                                                                         start_position, los_vehicles_tolerance,
                                                                         limit_vehicle_count=(start_position == 0.0))

    # if car is stuck in traffic jam, it will not move and its speed will be 0
    if level_of_service == float("inf"):
        # in case the vehicle is stuck in traffic jam just move the time
        return current_time + vehicle.frequency, vehicle.segment_position, SpeedMps(0.0)

    # Speed in m/s
    # Get highway type from the map
    try:
        highway_data = routing_map.current_network[current_segment.node_from][current_segment.node_to][0].get('highway', 'unclassified')
        if isinstance(highway_data, list):
            highway_type = highway_data[0]  # Take the primary highway type
        else:
            highway_type = highway_data
    except KeyError:
        highway_type = 'unclassified'  # Fallback if edge not found
    
    from ..utils import get_speed_limit_kph
    speed_limit_kph = get_speed_limit_kph(highway_type, vehicle.vehicle_type)
    segment_max_speed = speed_kph_to_mps(SpeedKph(speed_limit_kph * level_of_service))
    vehicle_max_speed = vehicle.max_speed_mps
    speed_mps = min(segment_max_speed, vehicle_max_speed)
    
    # Apply turning delay reduction
    if turn_type != 'straight':
        base_speed = speed_mps
        if vehicle.vehicle_type == 'car':
            if turn_type == 'turn':
                multiplier = 0.7
            else:  # u_turn
                multiplier = 0.5
        else:  # truck
            if turn_type == 'turn':
                multiplier = 0.6
            else:
                multiplier = 0.4
        if level_of_service < 0.5:
            multiplier *= 0.8
        speed_mps *= multiplier
        
        global turn_log_count
        if turn_log_count < 50:
            turn_log_count += 1
            logger.info(f"Turn delay: {vehicle.id}, {vehicle.vehicle_type}, prev {prev_segment.node_from}->{prev_segment.node_to}, next {current_segment.node_from}->{current_segment.node_to}, angle {abs_angle_deg:.1f}, {turn_type}, base {base_speed:.2f}, mult {multiplier:.2f}, final {speed_mps:.2f}, los {level_of_service:.2f}")
    
    if math.isclose(speed_mps, 0.0):
        # in case the vehicle is not moving, move the time and keep the previous position
        return current_time + vehicle.frequency, vehicle.segment_position, SpeedMps(0.0)

    frequency_s = vehicle.frequency.total_seconds()
    elapsed_m = frequency_s * speed_mps
    end_position = LengthMeters(start_position + elapsed_m)

    if end_position < current_segment.length:
        return (
            current_time + timedelta(seconds=frequency_s),
            SegmentPosition(index=segment_position.index, position=end_position),
            speed_mps
        )
    else:
        # The car has finished the segment, it stays at the end of it
        travel_distance_m = current_segment.length - start_position
        travel_time = travel_distance_m / speed_mps
        return (
            current_time + timedelta(seconds=travel_time),
            SegmentPosition(segment_position.index, current_segment.length),
            speed_mps
        )


def advance_vehicle(vehicle: Vehicle, departure_time: datetime,
                    gv_db: GlobalView, routing_map: Map, queues_manager: QueuesManager,
                    los_vehicles_tolerance: timedelta = timedelta(seconds=0)) -> List[FCDRecord]:
    """Advance a vehicle on a route."""

    current_time = departure_time + vehicle.time_offset
    fcds = []

    current_vehicle_index = vehicle.segment_position.index
    osm_route_part = vehicle.osm_route[current_vehicle_index:current_vehicle_index + 3]
    driving_route_part = routing_map.osm_route_to_py_segments(osm_route_part)

    vehicle_end_time, segment_pos, assigned_speed_mps = move_on_segment(
        vehicle, driving_route_part, current_time, gv_db, routing_map, los_vehicles_tolerance
    )

    segment_pos_old = vehicle.segment_position

    # check vehicle's first move
    if segment_pos.index == 0 and segment_pos.position == 0.0:
        # vehicle could not move on segment, skip this round and just advance the time
        vehicle.time_offset += vehicle.frequency
        return []

    # update the vehicle
    vehicle.time_offset += vehicle_end_time - current_time
    vehicle.set_position(segment_pos)

    # step_m = assigned_speed_mps * (vehicle.fcd_sampling_period / timedelta(seconds=1))
    # segment = driving_route[vehicle.segment_position.index]
    # dt = departure_time + vehicle.time_offset
    # logger.info(f"\n{dt} {vehicle.id} ({vehicle.start_distance_offset}) {segment.id} ({segment.length}) step: {step_m}")
    # logger.info(fcds)

    segment = driving_route_part[segment_pos.index - segment_pos_old.index]
    segment_old = driving_route_part[0]

    # fill in and out of the queues
    if (segment_pos_old.position == segment_old.length
            and segment_pos_old.index != vehicle.segment_position.index):
        # remove vehicle from outdated queue if it changed segment
        node_from, node_to = vehicle.osm_route[segment_pos_old.index], vehicle.osm_route[segment_pos_old.index + 1]
        queues_manager.remove_vehicle(vehicle, node_from, node_to)

    if vehicle.start_distance_offset == segment.length:
        if vehicle.next_node == vehicle.dest_node:
            # stop the processing in case the vehicle reached the end
            vehicle.active = False
            queues_manager.remove_inactive_vehicle(vehicle)

        elif segment_pos_old != vehicle.segment_position:
            # if the vehicle is at the end of the segment and was not there before, add it to the queue
            queues_manager.add_to_queue(vehicle)

    # NOTE: the segment position index may end out of segments
    if segment_pos.index < (len(vehicle.osm_route) - 1):
        fcds = generate_fcds(current_time, vehicle_end_time, segment_pos_old, segment_pos, assigned_speed_mps, vehicle,
                             driving_route_part, remains_active=vehicle.active)

    return fcds


def advance_waiting_vehicle(vehicle: Vehicle, routing_map: Map, departure_time: datetime) -> List[FCDRecord]:
    current_time = departure_time + vehicle.time_offset

    current_vehicle_index = vehicle.start_index
    osm_route_part = vehicle.osm_route[current_vehicle_index:current_vehicle_index + 3]
    driving_route_part = routing_map.osm_route_to_py_segments(osm_route_part)

    # in case the vehicle is not moving, move the time and keep the previous position
    vehicle_end_time = current_time + vehicle.frequency
    segment_pos = vehicle.segment_position
    assigned_speed_mps = SpeedMps(0.0)

    # update the vehicle
    vehicle.time_offset += vehicle_end_time - current_time

    fcds = generate_fcds(current_time, vehicle_end_time, segment_pos, segment_pos, assigned_speed_mps, vehicle,
                         driving_route_part, remains_active=True)
    return fcds


def generate_fcds(start_time: datetime, end_time: datetime, start_segment_position: SegmentPosition,
                  end_segment_position: SegmentPosition, speed: SpeedMps, vehicle: Vehicle,
                  driving_route_part: List[Segment], remains_active: bool) -> List[FCDRecord]:
    fcds = []

    step_m = speed * (vehicle.fcd_sampling_period / timedelta(seconds=1))

    # when both start and end positions are on the same segment
    current_position = start_segment_position.position
    current_time = start_time
    current_segment = driving_route_part[0]

    if current_position == current_segment.length and start_segment_position.index != end_segment_position.index:
        # when the vehicle finished the segment in the previous round, we will jump to the next segment
        current_segment = driving_route_part[1]
        current_position = 0

    while current_time + vehicle.fcd_sampling_period < end_time and \
            current_position + step_m < current_segment.length:
        current_position += step_m
        current_time += vehicle.fcd_sampling_period
        fcds.append(FCDRecord(
            datetime=current_time,
            vehicle_id=vehicle.id,
            segment=current_segment,
            start_offset=current_position,
            speed=speed,
            status=vehicle.status,
            active=True,
            vehicle_type=vehicle.vehicle_type
        ))

    # store end of the movement
    fcds.append(FCDRecord(
        datetime=end_time,
        vehicle_id=vehicle.id,
        segment=current_segment,
        start_offset=end_segment_position.position,
        speed=speed,
        status=vehicle.status,
        active=remains_active,
        vehicle_type=vehicle.vehicle_type
    ))
    return fcds


def advance_vehicles_with_queues(vehicles_to_be_moved: List[Vehicle], departure_time: datetime,
                                 gv_db: GlobalView, routing_map: Map, queues_manager: QueuesManager,
                                 los_vehicles_tolerance) -> Tuple[List[FCDRecord], bool]:
    fcds = []

    vehicles_moved = False
    vehicles_in_queues = dict()
    for vehicle in vehicles_to_be_moved:
        queue = queues_manager.queues[(vehicle.current_node, vehicle.next_node)]
        if vehicle.id not in queue:
            prev_pos = vehicle.segment_position
            new_fcds = advance_vehicle(vehicle, departure_time, gv_db, routing_map, queues_manager,
                                       los_vehicles_tolerance)
            fcds.extend(new_fcds)
            vehicles_moved = vehicles_moved or prev_pos != vehicle.segment_position
        else:
            vehicles_in_queues[vehicle.id] = vehicle

    for _, queue in queues_manager.queues.copy().items():
        queue_copy = list(queue)
        for vehicle_id in queue_copy:
            if vehicle_id not in vehicles_in_queues:
                break

            vehicle = vehicles_in_queues[vehicle_id]
            del vehicles_in_queues[vehicle_id]
            new_fcds = advance_vehicle(vehicle, departure_time, gv_db, routing_map, queues_manager,
                                       los_vehicles_tolerance)
            fcds.extend(new_fcds)
            was_moved = len(queue) == 0 or (vehicle_id != queue[0])
            vehicles_moved = vehicles_moved or was_moved
            if not was_moved:
                break

    for _, vehicle in vehicles_in_queues.items():
        new_fcds = advance_waiting_vehicle(vehicle, routing_map, departure_time)
        fcds.extend(new_fcds)

    # assert len(fcds) == len(vehicles_to_be_moved)
    return fcds, vehicles_moved
