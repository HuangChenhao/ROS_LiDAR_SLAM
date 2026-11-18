"""Nonblocking, bounded backend finalization for the Foxy SLAM supervisor.

Service acknowledgement and graph-file verification are reported separately
from map availability. Publishing an RTAB map is not a new optimization pass.
"""
import json
import os
import time


class SessionFinalizer:
    def __init__(self, node, algorithm, session, timeout=25.0):
        self.node = node
        self.session = session
        self.deadline = time.monotonic() + timeout
        self.jobs = []
        self.results = []
        self.client = None
        self.future = None
        self.done = False
        self.timer = None
        if algorithm == 'cartographer':
            from cartographer_ros_msgs.srv import FinishTrajectory, WriteState
            self.jobs = [
                (FinishTrajectory, '/finish_trajectory', {'trajectory_id': 0}, []),
                (WriteState, '/write_state',
                 {'filename': os.path.join(session, 'map.pbstream')}, ['map.pbstream'])]
        elif algorithm == 'slam_toolbox':
            from slam_toolbox.srv import SerializePoseGraph
            from nav_msgs.srv import GetMap
            self.jobs = [
                (SerializePoseGraph, '/slam_toolbox/serialize_map',
                 {'filename': os.path.join(session, 'pose_graph')},
                 ['pose_graph.posegraph', 'pose_graph.data']),
                (GetMap, '/slam_toolbox/dynamic_map', {}, [])]
        elif algorithm in ('rtabmap', 'rtabmap_rgbd'):
            from rtabmap_msgs.srv import PublishMap
            self.jobs = [(PublishMap, '/publish_map',
                          {'global_map': True, 'optimized': True, 'graph_only': False}, [])]
        self.timer = node.create_timer(0.1, self.poll)

    def _release_client(self):
        if self.future is not None and not self.future.done():
            self.future.cancel()
        if self.client is not None:
            self.node.destroy_client(self.client)
        self.client = self.future = None

    def poll(self):
        if self.done:
            return
        if time.monotonic() >= self.deadline:
            self.finish('timeout')
            return
        if not self.jobs:
            self.finish('complete')
            return
        service_type, name, fields, artifacts = self.jobs[0]
        try:
            if self.client is None:
                self.client = self.node.create_client(service_type, name)
            if self.future is None:
                if not self.client.service_is_ready():
                    return
                request = service_type.Request()
                for key, value in fields.items():
                    setattr(request, key, value)
                self.future = self.client.call_async(request)
                return
            if not self.future.done():
                return
            response = self.future.result()
            if response is None:
                raise RuntimeError('empty service response')
            if hasattr(response, 'status') and response.status.code != 0:
                raise RuntimeError(str(response.status.message))
            if hasattr(response, 'map'):
                if response.map.info.width == 0 or response.map.info.height == 0:
                    raise RuntimeError('backend returned an empty map')
                self.node.map_cb(response.map)
            missing = [p for p in artifacts if not os.path.isfile(os.path.join(self.session, p))
                       or os.path.getsize(os.path.join(self.session, p)) == 0]
            if missing:
                raise RuntimeError('graph files missing: ' + ', '.join(missing))
            self.results.append({'service': name, 'status': 'acknowledged', 'artifacts': artifacts})
            self.jobs.pop(0)
            self._release_client()
        except Exception as error:
            self.results.append({'service': name, 'status': 'failed', 'error': str(error)})
            self.node.get_logger().warn('SLAM finalization {}: {}'.format(name, error))
            self.jobs.clear()
            self.finish('failed')

    def finish(self, status):
        if self.done:
            return
        self.done = True
        if self.jobs:
            self.results.extend({'service': job[1], 'status': status} for job in self.jobs)
            self.jobs.clear()
        self._release_client()
        if self.timer is not None:
            self.timer.cancel()
            self.node.destroy_timer(self.timer)
            self.timer = None
        payload = {'status': status, 'operations': self.results}
        try:
            path = os.path.join(self.session, 'finalization.json')
            with open(path + '.tmp', 'w') as stream:
                json.dump(payload, stream, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(path + '.tmp', path)
        except OSError as error:
            self.node.get_logger().error('Cannot save finalization result: {}'.format(error))
        self.node.append_metadata('finalization_result: {}\n'.format(status))
        # Leave subscription callbacks time to consume the published final map.
        self.node.stop_deadline = time.monotonic() + 2.0


def snapshot_configuration(session, algorithm):
    """Keep the exact inputs needed to reproduce a run with the saved bag."""
    import hashlib
    import shutil
    paths = {
        'gmapping': '/root/yahboomcar_ros2_ws/software/library_ws/install/slam_gmapping/share/slam_gmapping/params/slam_gmapping.yaml',
        'cartographer': '/root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_nav/share/yahboomcar_nav/params/rosmaster_carto.lua',
        'slam_toolbox': '/root/rosmaster_tools/slam_toolbox_r2.yaml',
        'rtabmap': '/root/rosmaster_tools/rtabmap_r2.yaml',
        'rtabmap_rgbd': '/root/rosmaster_tools/rtabmap_rgbd_r2.yaml',
    }
    sources = [paths[algorithm], '/root/yahboomcar_ros2_ws/software/library_ws/install/robot_localization/share/robot_localization/params/ekf_x1_x3.yaml']
    folder = os.path.join(session, 'configuration')
    os.makedirs(folder, exist_ok=True)
    manifest = {}
    for source in sources:
        destination = os.path.join(folder, os.path.basename(source))
        shutil.copy2(source, destination)
        with open(destination, 'rb') as stream:
            manifest[source] = hashlib.sha256(stream.read()).hexdigest()
    with open(os.path.join(folder, 'sha256.json'), 'w') as stream:
        json.dump(manifest, stream, indent=2)
