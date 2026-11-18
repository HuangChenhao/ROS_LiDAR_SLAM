"""Exercise finalization failures without a ROS graph or robot hardware."""
import importlib.util
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('slam_finalization', Path(__file__).parents[1] / 'slam_finalization.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FakeService:
    class Request:
        pass


class Future:
    def __init__(self, response):
        self.response = response
    def done(self):
        return True
    def result(self):
        return self.response


class Node:
    def __init__(self, response=None, ready=True):
        self.messages = []
        self.requests = []
        self.response = response or types.SimpleNamespace()
        self.ready = ready
        self.stop_deadline = 0
    def create_timer(self, period, callback):
        return types.SimpleNamespace(cancel=lambda: None)
    def destroy_timer(self, timer):
        pass
    def create_client(self, srv, name):
        def call(request):
            self.requests.append((name, request))
            return Future(self.response)
        return types.SimpleNamespace(service_is_ready=lambda: self.ready, call_async=call)
    def destroy_client(self, client):
        pass
    def get_logger(self):
        return types.SimpleNamespace(warn=self.messages.append, error=self.messages.append)
    def append_metadata(self, msg):
        self.messages.append(msg)


class FinalizationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(self.tmp.cleanup)
        self.imports = patch.dict('sys.modules', {
            'rtabmap_msgs': types.ModuleType('rtabmap_msgs'),
            'rtabmap_msgs.srv': types.SimpleNamespace(PublishMap=FakeService),
            'slam_toolbox': types.ModuleType('slam_toolbox'),
            'slam_toolbox.srv': types.SimpleNamespace(SerializePoseGraph=FakeService),
            'nav_msgs': types.ModuleType('nav_msgs'),
            'nav_msgs.srv': types.SimpleNamespace(GetMap=FakeService)})
        self.imports.start()
        self.addCleanup(self.imports.stop)

    def test_acknowledgement_records_publish_not_optimization(self):
        node = Node()
        runner = module.SessionFinalizer(node, 'rtabmap', self.tmp.name)
        runner.poll()
        self.assertFalse(runner.done)
        runner.poll()
        runner.poll()
        self.assertTrue(runner.done)
        self.assertEqual(runner.results[0]['service'], '/publish_map')
        self.assertEqual(runner.results[0]['status'], 'acknowledged')
        self.assertTrue((Path(self.tmp.name) / 'finalization.json').exists())
        self.assertGreater(node.stop_deadline, 0)

    def test_missing_service_has_bounded_timeout(self):
        node = Node(ready=False)
        runner = module.SessionFinalizer(node, 'rtabmap', self.tmp.name)
        runner.poll()
        self.assertFalse(runner.done)
        self.assertEqual(node.requests, [])
        runner.deadline = 0
        runner.poll()
        self.assertTrue(runner.done)
        self.assertEqual(runner.results[0]['status'], 'timeout')

    def test_empty_serialize_response_is_not_proof_of_saved_graph(self):
        runner = module.SessionFinalizer(Node(), 'slam_toolbox', self.tmp.name)
        runner.poll()
        runner.poll()
        self.assertTrue(runner.done)
        self.assertEqual(runner.results[0]['status'], 'failed')
        self.assertIn('graph files missing', runner.results[0]['error'])

    def test_cancellation_is_idempotent(self):
        node = Node(ready=False)
        runner = module.SessionFinalizer(node, 'rtabmap', self.tmp.name)
        runner.finish('interrupted')
        runner.finish('complete')
        self.assertEqual(runner.results[0]['status'], 'interrupted')
        self.assertEqual(len(node.messages), 1)


if __name__ == '__main__':
    unittest.main()
