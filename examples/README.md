# Standalone synthetic example

`standalone.cpp` feeds synthetic coordinates near **10° N, 20° E** to the core. It does not open a serial port or use a dataset.

From the repository root, create `runtime/` and compile:

```bash
g++ -std=c++11 -O2 -Wall -Wextra -pedantic examples/standalone.cpp -o runtime/standalone
./runtime/standalone
```

Windows GCC users can name the output `runtime/standalone.exe`. Keep the compiler runtime DLL directory on PATH.

The example demonstrates shared GPS/gyro source time, integer-degree scaling, checking validity, and reading diagnostic weights. It is an API example, not a positioning benchmark.
