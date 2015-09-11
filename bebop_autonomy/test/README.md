## Running BIL test with the new catkin

TBA

## Running BIL test manually

```bash
$ cd bebop_ws/build/bebop_autonomy
$ make run_tests
```

## Debugging BIL test

```bash
$ cd bebop_ws/build/bebop_autonomy
$ make tests
$ rostest --text bebop_autonomy bebop_itl_test.test
```
