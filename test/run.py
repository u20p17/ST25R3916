"""Compile and exercise the real driver without Arduino, OpenKNX or HomeKey."""
from pathlib import Path
import os, subprocess, tempfile
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='st25r3916-test-') as temp:
    binary=str(Path(temp)/'reader-test')
    for timing in (False, True):
        flags = ['-DST25R3916_TIMING_TEST'] if timing else []
        subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-fsanitize=address,undefined',
        *flags,'-DARDUINO_ARCH_ESP32','-I'+str(root/'test/stubs'),'-I'+str(root/'test/support'),
        '-I'+str(root/'src'),str(root/'test/reader_test.cpp'),str(root/'src/ST25R3916.cpp'),
        '-o',binary],check=True)
        for mode in ('baseline', 'fast'):
            subprocess.run([binary,mode],check=True)
