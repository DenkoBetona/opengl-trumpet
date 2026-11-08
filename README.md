# 3D тромпет

Курсов проект по Компютърна графика (BCSE51), ТУ – София.

<img width="1400" height="550" alt="screenshot" src="https://github.com/user-attachments/assets/87a7085b-7290-4172-9814-736d37404f20" />

## Управление

| Клавиши | Действие |
|---|---|
| A S D F G H | обертони 2, 3, 4, 5, 6, 8 |
| J K L | вентили 1, 2, 3 |
| Tab | превключване към режим „пиано“ |
| Z … M, Q … I | две октави (режим „пиано“) |
| ↑ ↓ | октава (C2 – C5) |
| мишка | въртене (влачене) и мащабиране (колелце) |
| Esc | изход |

## Компилиране

Нужни са CMake, Git и C++17 компилатор (Visual Studio с „Desktop development with C++“ или GCC).
CMake сам изтегля GLFW и miniaudio.

На Windows: двоен клик на `build.bat` – компилира и стартира програмата.

```
cmake -B build
cmake --build build --config Release
```

Стартиране: `build\Release\trumpet.exe` (Windows) или `./build/trumpet` (Linux).

## Използвани библиотеки

- [GLFW](https://www.glfw.org) – прозорец, OpenGL контекст и вход
- [miniaudio](https://miniaud.io) – извеждане на звук
