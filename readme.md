# pike

## Подготовка

Для сборки используются:
1. CMake
2. Visual Studio 2015
3. Qt 5.12 с набором msvc2015_64, опционально Qt Creator/Qt Designer для разработки UI

## Сборка

Компиляция для Windows идёт под 64-битную архитектуру, п.ч. Qt поставляет для VS 2015 только 64-битный набор (тулчейн) msvc2015_64. Сама Qt рекомендует собирать 32-bit в VS 2015 с использованием набора msvc2017.

```sh
md build
cd build
cmake -G "Visual Studio 14 2015" -A x64 -D Qt5_DIR=${VS_QT}/lib/cmake/Qt5 ..
cmake --build . --target ALL_BUILD --config Release
```

где ${VS_QT} - путь к тулчейну QT (например, C:/Qt/Qt5.12.3/5.12.3/msvc2015_64).

### Сборка с тестами

```sh
cmake -G "Visual Studio 14 2015" -A x64 -D Qt5_DIR=${VS_QT}/lib/cmake/Qt5 -D BUILD_TESTING=ON ..
cmake --build . --target ALL_BUILD --config Release
ctest
```

## Дистрибутив

Дистрибутив должен содержать следующие файлы:
- e440.bio (или другой биос для используемой АЦП) (из `_ext/lcomp/bios`)
- inclinometer.tbl (из `_misc/deploy`)
- lcomp64.dll (из `_ext/lcomp/lib`)
- pike.json (из `_misc/deploy`)
- pike-qwidgets.exe
- Qt5Core.dll
- Qt5Gui.dll
- Qt5Network.dll
- Qt5Widgets.dll

Для сбора всех необходимых файлов Qt рекомендуется использовать windeployqt.exe (для примера см. Korsar3RPi). Также в системе должен быть установлен VCRedist 2015 x64.

## Архитектура ПО

Термины:
- Сечение - измерение глубины в одной плоскости и результат такого измерения.

Проект разбит на библиотеки (по логическим уровням):
- dc - устройство сбора данных (Л-Кард и DummyDaq).
- devices - устройства, входящие в "Пику" (двигатели, одометр, инклинометр, глубинометр и "концевые" датчики).
- logic - логика работы/операции (определение пройденного расстояния, положения в пространстве и текущих показаний глубины и "концевых" датчиков, управление с помощью джойстика через UDP, измерение глубины и конфигурация).
- modules - MVP-модули окон приложения (main). Частично логика перемещения и вращения измерительного блока.
- pike-qwidgets - GUI-приложение на основе QtWidgets.

Все устройства и операции "развязаны" через интерфейсы (абстрактные классы). Разделение логики и GUI через MVP.

### Реализация devices

- Depthometer - Датчик расстояния/глубины (FASTUS CD22). Подключение через COM-порт.
- Ender - "Концевой" датчик. Подключение через входную линию ТТЛ.
- Inclinometer - Инклинометр. Подключение через АЦП - с датчика приходят два PWM-сигнала. Для упрощения считаем по ним СКЗ и используем таблицы пересчёта (вместо расчёта коэффициента заполнения).
- Mover - Перемещение вперёд-назад с помощью коллекторного двигателя. Подключение через выходные линии ТТЛ - PWM и Dir. Ожидаемая частота импульсов PWM 10 кГц, но мы используем режим максимальной скорости (единичный коэфф. заполнения).
- Odometer - Одометр/курвиметр/датчик пройденного расстояния (ЛИР-119). Подключение через АЦП - с датчика приходят два strobe-сигнала.
- Rotator - Вращение измерительного блока с помощью шагового двигателя (TI DRV8825). Подключение через выходные линии ТТЛ - Enable, Step, Dir и M0.

### Реализация logic

- Conf - Конфигурация ПО через файл pike.json.
- OngoingReader - Режим измерения пройденного расстояния, положения в пространстве, состояния "концевых" датчиков и глубины. Измерение идёт постоянно, кроме "концевых" датчиков и глубины, которые "приостанавливаются" на время измерения сечения (Slicer).
- Pike - "Пика". По идее сюда нужно перенести управление перемещением, вращением и измерением сечения из MainPresenterImpl.
- RemoteServer - Режим приёма событий от джойстка, подключенного на удалённом копьютере, через UDP.
- Slicer - Режим измерения сечения - поворот измерительного блока в крайнее левое положение, измерение положения в пространстве и циклическое измерение глубины (и инкремент угла) до крайнего правого положения.

## TODO

- камеры
- сохранение видео и фото
- джойстик (через pike-remote)
- сохранение "сечения" (SliceMsrMapper)
- логирование?
- перед запуском на устройстве:
  - "активное" состояние "концевого" датчика - 0 или 1? (нужен тест)
  - одометр: вперёд/назад - запаздывание фронтов (нужен тест)
  - инклинометр: неполная блок-схема (СНН)
  - инклинометр: СКЗ? (нужен тест)
  - инклинометр: файл с таблицей пересчёта СКЗ в угол
