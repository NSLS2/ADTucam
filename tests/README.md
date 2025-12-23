# ADTucam Unit Tests

Simple unit testing for the ADTucam EPICS driver using Google Test/Mock.


## Building Tests

From the project root:

```bash
# Build tests
make UNIT_TESTING=YES

# Or build just tests
cd tests && make
```

## Running Tests

```bash
# Run all tests
cd tests && make test

# Run with filter
cd tests && ./O.linux-x86_64/test_adtucam --gtest_filter="*Temperature*"

# Run with verbose output
cd tests && ./O.linux-x86_64/test_adtucam --gtest_verbose
```

## Files

- `test_ADTucam.cpp` - Main test suite
- `MockCameraSDK.h/cpp` - Mock camera SDK for testing
- `Makefile` - Simple EPICS build configuration

## Adding Tests

Add new tests to `test_ADTucam.cpp`:

```cpp
TEST_F(ADTucamTest, YourNewTest) {
  // Set up mock expectations
  mockSDK->SetupSuccessfulConnection();

  // Create driver and test
  ADTucam* driver = new ADTucam("TEST_PORT", 0, mockSDK);
  // ... test logic ...
  delete driver;
}
```
