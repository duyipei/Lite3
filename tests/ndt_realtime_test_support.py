"""Compile extracted production C++ against explicit offline test doubles."""
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / 'vendor_patch/hdl_localization_app.hpp'


def method(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 1
    for i in range(opening + 1, len(source)):
        depth += (source[i] == '{') - (source[i] == '}')
        if not depth:
            return source[start:i + 1]
    raise ValueError('unclosed method: ' + signature)


def section(source, name):
    return source.split('// BEGIN_' + name, 1)[1].split('// END_' + name, 1)[0]


def compile_cpp(directory, name, source, eigen=False):
    directory = Path(directory)
    cpp = directory / (name + '.cpp')
    exe = directory / name
    cpp.write_text(source)
    args = ['g++', '-std=c++14', '-O1', '-Wall', '-Wextra']
    if eigen:
        args.append('-I/usr/include/eigen3')
    result = subprocess.run(args + [str(cpp), '-o', str(exe)], capture_output=True, text=True)
    if result.returncode:
        raise AssertionError(result.stderr)
    return exe
