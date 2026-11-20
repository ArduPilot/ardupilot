#!/usr/bin/env python3

'''
Unittests for the LTO linker map support in extract_features.py

AP_FLAKE8_CLEAN
'''

import os
import shutil
import subprocess
import tempfile
import unittest

from unittest.mock import patch

from extract_features import ExtractFeatures

MAP = '''Memory Configuration

Cross Reference Table

Symbol                                            File
AP_Airspeed::AP_Airspeed()                        AP_Airspeed.cpp.3.o (symbol from plugin)
                                                  ArduCopter/Copter.cpp.99.o (symbol from plugin)
AP_Compass_AK8963::probe(AP_HAL::OwnPtr<AP_HAL::Device>, Rotation) AP_Compass_AK8963.cpp.0.o (symbol from plugin)
AP_Compass_AK8963::probe_mpu9250(AP_HAL::OwnPtr<AP_HAL::Device>, Rotation) /data/My Build/AP_Compass.cpp.0.o
AP_Compass_AK8963::probe_ak09916(AP_HAL::OwnPtr<AP_HAL::Device>, Rotation) My Build/AK09916.o
AP_AK09916::AP_AK09916()                          My Build/AK09916.o
AC_Fence::check(bool)                             lib/libArduCopter_libs.a(AC_Fence.cpp.0.o)
                                                  ArduCopter/fence.cpp.0.o
AP_Mount::AP_Mount()                              /data/My Build/AP_Mount.cpp.0.o (symbol from plugin)
_ZN6AP_RPMC2Ev                                    AP_RPM.cpp.0.o (symbol from plugin)
'''


class TestExtractFeaturesMap(unittest.TestCase):

    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.elf = os.path.join(self.tmpdir.name, 'arducopter')

    def tearDown(self):
        self.tmpdir.cleanup()

    def write_map(self, path):
        with open(path, 'w') as f:
            f.write(MAP)

    def extract(self, elf_symbols, map_filename=None, section=None):
        ef = ExtractFeatures(self.elf, nm='nm', map_filename=map_filename)
        symbols = ExtractFeatures.Symbols()
        for name in elf_symbols:
            symbols.add(name, {'size': 0})
        with patch.object(ExtractFeatures, 'read_elf_section', return_value=section), \
             patch.object(ExtractFeatures, 'extract_symbols_from_elf', return_value=symbols), \
             patch.object(ExtractFeatures, 'extract_strings_from_elf', return_value=[]):
            (compiled_in, _) = ef.extract()
        return set(compiled_in)

    def test_map_symbols_parsed(self):
        self.write_map(self.elf + '.map')
        symbols = ExtractFeatures.Symbols()
        ExtractFeatures(self.elf, nm='nm').add_symbols_from_map(symbols, self.elf + '.map')
        self.assertEqual(set(symbols.symbols.keys()), {
            'AP_Airspeed::AP_Airspeed()',
            'AP_Compass_AK8963::probe(AP_HAL::OwnPtr<AP_HAL::Device>, Rotation)',
            'AC_Fence::check(bool)',
            'AP_Compass_AK8963::probe_mpu9250(AP_HAL::OwnPtr<AP_HAL::Device>, Rotation)',
            'AP_Compass_AK8963::probe_ak09916(AP_HAL::OwnPtr<AP_HAL::Device>, Rotation)',
            'AP_AK09916::AP_AK09916()',
            'AP_Mount::AP_Mount()',
            'AP_RPM::AP_RPM()',
        })

    def test_cxxfilt_beside_nm(self):
        for (nm, cxxfilt) in [('arm-none-eabi-nm', 'arm-none-eabi-c++filt'),
                              ('/opt/bin/arm-none-eabi-nm.exe', '/opt/bin/arm-none-eabi-c++filt.exe'),
                              ('arm-none-eabi-gcc-nm', 'arm-none-eabi-c++filt'),
                              ('llvm-nm-18', 'c++filt')]:
            self.assertEqual(ExtractFeatures(self.elf, nm=nm).cxxfilt, cxxfilt)

    def test_missing_cxxfilt_fails(self):
        self.write_map(self.elf + '.map')
        ef = ExtractFeatures(self.elf, cxxfilt='/no/such/c++filt')
        with self.assertRaises(ValueError):
            ef.add_symbols_from_map(ExtractFeatures.Symbols(), self.elf + '.map')

    def test_lto_uses_map_beside_elf(self):
        self.write_map(self.elf + '.map')
        features = self.extract([ExtractFeatures.LTO_MARKER])
        self.assertIn('AP_AIRSPEED_ENABLED', features)
        self.assertIn('AP_FENCE_ENABLED', features)

    def test_lto_uses_given_map(self):
        map_filename = os.path.join(self.tmpdir.name, 'other.map')
        self.write_map(map_filename)
        features = self.extract([ExtractFeatures.LTO_MARKER], map_filename=map_filename)
        self.assertIn('AP_AIRSPEED_ENABLED', features)

    def test_lto_without_map_fails(self):
        with self.assertRaises(ValueError):
            self.extract([ExtractFeatures.LTO_MARKER])

    def test_map_without_cref_fails(self):
        with open(self.elf + '.map', 'w') as f:
            f.write('Memory Configuration\n')
        with self.assertRaises(ValueError):
            self.extract([ExtractFeatures.LTO_MARKER])

    def test_embedded_features_used(self):
        features = self.extract([], section=b'AP_FENCE_ENABLED\n!AP_AIRSPEED_ENABLED\nNOT_AN_OPTION\n')
        self.assertEqual(features, {'AP_FENCE_ENABLED'})

    def test_read_elf_section(self):
        objcopy = shutil.which('objcopy')
        true = shutil.which('true')
        if objcopy is None or true is None:
            self.skipTest('needs objcopy')
        src = os.path.join(self.tmpdir.name, 'features.txt')
        with open(src, 'w') as f:
            f.write('AP_FENCE_ENABLED\n')
        subprocess.run([objcopy, '--add-section', ExtractFeatures.FEATURES_SECTION + '=' + src,
                        true, self.elf], check=True)
        ef = ExtractFeatures(self.elf)
        self.assertEqual(ef.read_elf_section(self.elf, ExtractFeatures.FEATURES_SECTION),
                         b'AP_FENCE_ENABLED\n')
        self.assertIsNone(ef.read_elf_section(self.elf, '.no_such_section'))

    def test_no_lto_ignores_map(self):
        self.write_map(self.elf + '.map')
        features = self.extract([])
        self.assertNotIn('AP_AIRSPEED_ENABLED', features)


if __name__ == '__main__':
    unittest.main()
