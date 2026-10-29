"""Publication checks: preserved supplied bytes and frozen submission identity."""
from pathlib import Path
import hashlib,json,unittest
ROOT=Path(__file__).resolve().parents[1]
class RepositoryTests(unittest.TestCase):
    def test_supporting_files_match_supplied_manifest(self):
        manifest=json.loads((ROOT/'review/supporting_file_manifest.json').read_text())
        for name,expected in manifest['sha256'].items():
            with self.subTest(path=name):self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),expected)
    def test_submission_matches_frozen_verified_version(self):
        record=json.loads((ROOT/'review/verification_checks.json').read_text())
        self.assertEqual(hashlib.sha256((ROOT/'student_controller3.py').read_bytes()).hexdigest(),record['controller_sha256'])
    def test_all_symlinks_are_local_and_resolve(self):
        for p in ROOT.rglob('*'):
            if '.git' in p.parts or not p.is_symlink():continue
            self.assertTrue(p.exists(),str(p))
            self.assertTrue(p.resolve().is_relative_to(ROOT),str(p))
if __name__=='__main__':unittest.main()
