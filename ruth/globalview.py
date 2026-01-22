from collections import defaultdict
from datetime import timedelta
from typing import Dict, List, Optional, Set, TYPE_CHECKING

from .data.segment import SegmentId, SpeedKph
from .vehicle_types import DEFAULT_VEHICLE_CLASSES

if TYPE_CHECKING:
    from .simulator.simulation import FCDRecord


class GlobalView:
    def __init__(self):
        self.fcd_by_segment: Dict[SegmentId, List[FCDRecord]] = defaultdict(list)

        """
        These variables keep track of which segments have been modified since
        the last call to "take_segment_speeds". We use this to only update segments
        that were modified in the map.
        """
        # vehicle ID -> segment ID
        self.car_to_segment: Dict[int, SegmentId] = {}
        self.modified_segments: Set[SegmentId] = set()

    def add(self, fcd: "FCDRecord"):
        self.fcd_by_segment[fcd.segment.id].append(fcd)
        self.modified_segments.add(fcd.segment.id)
        old_segment = self.car_to_segment.get(fcd.vehicle_id, None)
        if old_segment is not None:
            if old_segment != fcd.segment.id:
                self.modified_segments.add(old_segment)
                self.car_to_segment[fcd.vehicle_id] = fcd.segment.id
        else:
            self.car_to_segment[fcd.vehicle_id] = fcd.segment.id

    def load_ahead(self, datetime, segment_id, tolerance=None, vehicle_id=-1,
                   vehicle_offset_m=0):
        # sums the PCU and occupied lengths (length + standstill gap) of all vehicles ahead of the vehicle with vehicle_id at the given segment_id at given time range
        # if case vehicle_id not set, then all vehicles are summed

        tolerance = tolerance if tolerance is not None else timedelta(seconds=0)
        vehicles_ahead = {}
        for fcd in self.fcd_by_segment.get(segment_id, []):
            if datetime - tolerance <= fcd.datetime <= datetime + tolerance:
                if fcd.vehicle_id != vehicle_id and fcd.start_offset > vehicle_offset_m:
                    if fcd.vehicle_id not in vehicles_ahead:
                        vehicle_params = DEFAULT_VEHICLE_CLASSES.get(fcd.vehicle_type, DEFAULT_VEHICLE_CLASSES["car"])
                        vehicles_ahead[fcd.vehicle_id] = vehicle_params
        total_pcu = sum(p.pcu for p in vehicles_ahead.values())
        total_occupied_length = sum(p.length_m + p.standstill_gap_m for p in vehicles_ahead.values())
        return total_pcu, total_occupied_length, vehicles_ahead

    def level_of_service_in_front_of_vehicle(self, datetime, segment, vehicle_id=-1,
                                             vehicle_offset_m=0, tolerance=None, limit_vehicle_count=False):
        mile = 1609.344  # meters
        # density of vehicles per mile with ranges of level of service
        # https://transportgeography.org/contents/methods/transport-technical-economic-performance-indicators/levels-of-service-road-transportation/
        ranges = [
            ((0, 12), (0.0, 0.2)),
            ((12, 20), (0.2, 0.4)),
            ((20, 30), (0.4, 0.6)),
            ((30, 42), (0.6, 0.8)),
            ((42, 67), (0.8, 1.0))]

        sum_pcu, sum_occupied_length, vehicles_ahead = self.load_ahead(datetime, segment.id, tolerance,
                                         vehicle_id, vehicle_offset_m)

        available_length = segment.length - vehicle_offset_m
        if limit_vehicle_count and vehicle_offset_m == 0.0 and segment.length >= 10.0:
            vehicles_occupied_length = sum_occupied_length
            if vehicles_occupied_length > available_length:
                # Count cars and trucks
                car_count = sum(1 for p in vehicles_ahead.values() if p.name == 'car')
                truck_count = sum(1 for p in vehicles_ahead.values() if p.name == 'truck')
                print(f"JAM TRIGGERED: segment {segment.id}, length={segment.length:.2f}, available={available_length:.2f}, lanes={segment.lanes}, vehicles={len(vehicles_ahead)}, sum_occupied={vehicles_occupied_length:.2f}, cars={car_count}, trucks={truck_count}")
                return float("inf")

        # NOTE: the ending length is set to avoid massive LoS increase at the end of the segments and also on short
        # segments, can be replaced with different LoS ranges for different road types in the future
        ending_length = 200
        rest_segment_length = segment.length - vehicle_offset_m

        # rescale density using PCU (effective vehicle count)
        if rest_segment_length < ending_length:
            n_vehicles_per_mile = sum_pcu * mile / ending_length
        else:
            n_vehicles_per_mile = sum_pcu * mile / (rest_segment_length * segment.lanes)

        los = float("inf")  # in case the vehicles are stuck in traffic jam
        for (low, high), (m, n) in ranges:
            if n_vehicles_per_mile < high:
                d = high - low  # size of range between two densities
                los = m + ((
                                       n_vehicles_per_mile - low) * 0.2 / d)  # -low => shrink to the size of the range
                break

        # reverse the level of service 1.0 means 100% LoS, but the input table defines it in reverse
        return los if los == float("inf") else 1.0 - los

    def level_of_service_in_time_at_segment(self, datetime, segment):
        return self.level_of_service_in_front_of_vehicle(datetime, segment, -1, 0, None)

    def take_segment_speeds(self) -> Dict[SegmentId, Optional[SpeedKph]]:
        """
        Returns all segments that have been modified since the last call to this
        method, along with their current speeds.
        """
        speeds = {}
        for segment_id in self.modified_segments:
            speeds[segment_id] = self.get_segment_speed(segment_id)
        self.modified_segments.clear()
        return speeds

    def get_segment_speed(self, segment_id: SegmentId) -> Optional[SpeedKph]:
        speeds = {}
        by_segment = list(self.fcd_by_segment[segment_id])
        by_segment.sort(key=lambda fcd: fcd.datetime)
        for fcd in by_segment:
            speeds[fcd.vehicle_id] = fcd.speed
        speeds = list(speeds.values())
        if len(speeds) == 0:
            return None
        return SpeedKph(sum(speeds) / len(speeds))

    def drop_old(self, dt_threshold):
        for (segment_id, old_fcds) in self.fcd_by_segment.items():
            new_fcds = [fcd for fcd in self.fcd_by_segment[segment_id] if fcd.datetime >= dt_threshold]
            if len(new_fcds) != len(old_fcds):
                self.fcd_by_segment[segment_id] = new_fcds
                # If the FCDs for the segments have changed, we need to update modified_segments
                self.modified_segments.add(segment_id)
