import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from preview import Feed


class PreviewTests(unittest.TestCase):
    def test_frames_are_throttled_but_contacts_survive_case_changes(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'frame.json'
            now = Mock(side_effect=[0, .01, .02, .03])
            feed = Feed(path, now=now)
            sample = Mock(return_value={'base': [1, 2, 3]})
            feed.publish('first', 'rubik_cube', sample, force=True)
            feed.publish('first', 'rubik_cube', sample)
            self.assertEqual(sample.call_count, 1)
            contact = {'finger': 'right', 'point': [1.2, 2, 3]}
            feed.publish('first', 'rubik_cube', sample, contact=contact)
            self.assertEqual(json.loads(path.read_text())['contact'], contact)
            feed.publish('second', 'dinosaur', sample, force=True)
            frame = json.loads(path.read_text())
            self.assertEqual(frame['id'], 'second')
            self.assertIsNone(frame['contact'])
            self.assertEqual(frame['hits'], {'first': contact})
            self.assertEqual(frame['sequence'], 3)
            self.assertEqual(list(Path(folder).iterdir()), [path])


if __name__ == '__main__':
    unittest.main()
