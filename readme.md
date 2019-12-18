# pike

## Build

Для сборки используется CMake, VS2015. Внешние зависимости - Qt5.

```
md build
cd build
cmake -G"Visual Studio 14 2015" -Ax64 -DQt5_DIR=${VS_QT}/lib/cmake/Qt5 ..
cmake --build . --target ALL_BUILD --config Release
```

где ${VS_QT} - путь к тулчейну QT (например, C:/Qt/Qt5.12.3/5.12.3/msvc2015_64).

### Build and run tests

```
cmake -G"Visual Studio 14 2015" -Ax64 -DQt5_DIR=${VS_QT}/lib/cmake/Qt5 -DBUILD_TESTING=ON ..
cmake --build . --target ALL_BUILD --config Release
# или так, чтобы собрать только тесты
cmake --build . --target RUN_TESTS --config Release
ctest
```
