# ruth/data/hdf5_writer.py
from datetime import datetime
from typing import List
import h5py
import numpy as np
from pathlib import Path


# Extended compound dtype for HDF5 (adds vehicle_type as fixed-length ASCII)
compound_dtype = np.dtype([
    ("timestamp", np.int64),  # Timestamp in seconds
    ("node_from", np.int64),
    ("node_to", np.int64),
    ("segment_length", np.int32),
    ("vehicle_id", np.int64),
    ("start_offset_m", np.float32),
    ("speed_mps", np.float32),
    ("active", np.bool_),
    ("vehicle_type", "S10"),  # new field: fixed-length ASCII
])

class HDF5Writer:
    def __init__(self, filename, dtype=None, od_parquet_path: str = "benchmarks/od-matrices/vehicles_with_types.parquet"):
        self.filepath = Path(filename)
        # open file for append/create
        self.file = h5py.File(filename, "a")

        # load od -> vehicle_type mapping (best-effort)
        self.od_map = {}
        try:
            import pandas as pd
            odp = Path(od_parquet_path)
            if odp.exists():
                od = pd.read_parquet(odp)
                if "id" in od.columns and "vehicle_type" in od.columns:
                    od = od.rename(columns={"id": "vehicle_id"})
                if "vehicle_id" in od.columns and "vehicle_type" in od.columns:
                    # ensure integer keys
                    self.od_map = od.set_index("vehicle_id")["vehicle_type"].to_dict()
                else:
                    # unable to find expected columns; leave empty
                    self.od_map = {}
        except Exception:
            # Do not fail if pandas/tables not available or file missing.
            self.od_map = {}

        # Determine whether we can use an existing dataset or must create a new one.
        # If existing fcd dataset matches our new dtype, use it.
        # If existing fcd exists but lacks vehicle_type, create 'fcd_with_type' dataset instead.
        if 'fcd' not in self.file or not isinstance(self.file['fcd'], h5py.Dataset):
            target_name = 'fcd'
        else:
            existing_dtype = self.file['fcd'].dtype
            if existing_dtype == compound_dtype:
                target_name = 'fcd'
            else:
                # create a new dataset name so we don't overwrite old format
                target_name = 'fcd_with_type'
                # if fcd_with_type already present, reuse it; otherwise we'll create it below

        # create or open dataset with our compound dtype
        if target_name not in self.file:
            self.dataset = self.file.create_dataset(
                target_name,
                shape=(0,),
                maxshape=(None,),
                dtype=compound_dtype,
                chunks=True,
                compression="gzip"
            )
        else:
            self.dataset = self.file[target_name]

        self.index = self.dataset.shape[0]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def save_computational_time(self, computational_time: float):
        if 'computational_time' not in self.file.attrs:
            self.file.attrs['computational_time'] = computational_time
            self.file.flush()

    def save_map(self, routing_map, departure_time: datetime):
        if 'bbox' not in self.file.attrs:
            self.file.attrs['bbox'] = tuple(routing_map.bbox.get_coords())
        if 'download_date' not in self.file.attrs:
            self.file.attrs['download_date'] = str(routing_map.download_date)
        if 'departure_time' not in self.file.attrs:
            self.file.attrs['departure_time'] = departure_time.isoformat()
        self.file.flush()

    def append_file(self, buffer: List):
        # Build a structured numpy array including vehicle_type
        def _get_vtype(vid):
            try:
                return self.od_map.get(int(vid), "unknown")
            except Exception:
                return "unknown"

        # Compose tuples that match compound_dtype
        rows = []
        for fcd in buffer:
            ts = int(fcd.datetime.timestamp())  # seconds (consistent with earlier code)
            node_from = fcd.segment.node_from
            node_to = fcd.segment.node_to
            seg_len = fcd.segment.length
            vid = int(fcd.vehicle_id)
            start_off = float(fcd.start_offset)
            speed = float(fcd.speed)
            active = bool(fcd.active)
            vtype = _get_vtype(vid)
            # ensure bytes fit S10
            vtype_b = vtype.encode("ascii")[:10] if isinstance(vtype, str) else str(vtype).encode("ascii")[:10]
            rows.append((ts, node_from, node_to, seg_len, vid, start_off, speed, active, vtype_b))

        if len(rows) == 0:
            return 0

        data = np.array(rows, dtype=compound_dtype)

        # Append to HDF5 dataset
        data_len = len(data)
        self.dataset.resize((self.index + data_len,))
        self.dataset[self.index:self.index + data_len] = data
        self.index += data_len
        self.file.flush()
        return data_len

    def close(self):
        self.file.close()


def get_edge_id_from_data(edge_data):
    """Extract the HDF5 edge ID from edge data."""
    return edge_data.get('id', 0)


def save_graph_to_hdf5(graph, filepath):
    """Save the networkx graph to HDF5 and return osm_to_hdf_map_ids dict."""
    import networkx as nx
    
    with h5py.File(filepath, 'w') as f:
        # Create datasets for nodes and edges
        nodes = list(graph.nodes())
        edges = list(graph.edges(data=True))
        
        # Assign HDF IDs to edges
        osm_to_hdf_map_ids = {}
        hdf_id = 0
        for u, v, data in edges:
            if 'id' not in data:
                data['id'] = hdf_id
                osm_to_hdf_map_ids[(u, v)] = hdf_id  # Assuming (u,v) is the osm id
                hdf_id += 1
        
        # Save nodes
        f.create_dataset('nodes', data=nodes)
        
        # Save edges with data
        # For simplicity, save as JSON strings or something, but since it's HDF5, perhaps save attributes
        # This is a basic implementation; may need to be expanded
        
        return osm_to_hdf_map_ids
