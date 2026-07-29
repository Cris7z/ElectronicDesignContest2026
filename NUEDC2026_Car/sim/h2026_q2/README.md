# H2026 Q2 host tests

Run on Windows from this directory:

```powershell
mingw32-make clean
mingw32-make test
```

The build uses GCC C11 with `-Wall -Wextra -Werror -pedantic`. The simulation
values in `test_h2026_q2.c` are explicit test fixtures, not measured vehicle
parameters.
