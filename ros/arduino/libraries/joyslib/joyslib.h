// расчёт сигнала, который надо подавать на колёса,
// по положению джойстика.

#ifndef _JOYSLIB_H_
#define _JOYSLIB_H_

#include<math.h>

// описание параметров сигналов колёс и сигналов джойстика
// для функций преобразования
struct joystick_to_wheels_transform_params{
  float middle_joystick_x; // Середина диапазона горизонтальной координаты джойстика
  float amplitude_joystick_x; // Амплитуда значений горизонтальной координаты джойстика
  float middle_joystick_y; // Середина диапазона вертикальной координаты джойстика
  float amplitude_joystick_y; // Амплитуда значений вертикальной координаты джойстика
  float middle_wheel_signal; // Середина диапазона сигналов колёс
  float amplitude_wheel_signal; // Амплитуда сигналов колёс
};

// Задать параметры преобразования сигналов джойстика в сигналы колёс и обратно.
// min_joystick_x, max_joystick_x --- максимальное и минимальное значения
// горизонтальной координаты джойстика. Должны быть измерены до начала работы;
// min_joystick_y, max_joystick_y --- максимальное и минимальное значения
// вертикальной координаты джойстика. Также должны быть измерены до начала работы;
// min_wheel_signal, max_wheel_signal --- максимальное и минимальное значения сигналов,
// поступающих на колёса.
// return --- 0, если параметры успешно заданы;
// -1 при ошибке в параметрах (минимальное значение больше максимального или равно ему);
// -2 при нулевом указателе на структуру хранения.
int set_joystick_to_wheels_transform_params(struct joystick_to_wheels_transform_params *p,
					    int min_joystick_x,
					    int max_joystick_x,
					    int min_joystick_y,
					    int max_joystick_y,
					    int min_wheel_signal,
					    int max_wheel_signal);

// Получить значения сигналов для колёс робота по положению джойстика.
// *p --- структура, описавающая параметры сигналов;
// joystick_x --- горизонтальная координата джойстика (увеличивается слева направо);
// joystick_y --- вертикальная координата джойстика (увеличивается снизу вверх);
// *left_wheel_signal, *right_wheel_signal --- вычисленные сигналы для левого и правого колёс
// (максимальное значение соответствует движению вперёд с максимальной скоростью);
// return --- 0 при нормальном завершении, <0 при ошибке:
// -1, если входные значения за пределами допустимого диапазона, указанного в *p;
// -2, если указатель на структуру хранения нулевой.
int get_signals_by_joystick_coords(const struct joystick_to_wheels_transform_params *p,
				   int joystick_x, int joystick_y,
				   int *left_wheel_signal, int *right_wheel_signal);


// Получить положение джойстика по значению сигналов колёс робота.
// *p --- структура, описавающая параметры сигналов;
// left_wheel_signal, right_wheel_signal --- сигналы, подаваемые на каждое из колёс;
// joystick_x --- вычисленная горизонтальная координата джойстика;
// joystick_y --- вычисленная вертикальная координата джойстика;
// return --- 0 при нормальном завершении, <0 при ошибке (-1, если входные значения за пределами
// допустимого диапазона, указанного в *p; -2, если указатель на структуру
// хранения нулевой).
int get_joystick_coords_by_signals(const struct joystick_to_wheels_transform_params *p,
				   int left_wheel_signal, int right_wheel_signal,
				   int *joystick_x, int *joystick_y);

//----------------------------------------------------------
struct TJoystick
{
  joystick_to_wheels_transform_params jst;
  TJoystick(int min_joystick_x, int max_joystick_x,
            int min_joystick_y, int max_joystick_y,
            int min_wheel_signal, int max_wheel_signal)
  { 
    set_joystick_to_wheels_transform_params(&jst, min_joystick_x, max_joystick_x, min_joystick_y, max_joystick_y,
                                            min_wheel_signal, max_wheel_signal); 
  }

  // Получить значения сигналов для колёс робота по положению джойстика.
  int j2u(int joystick_x, int joystick_y,
	  int *left_wheel_signal, int *right_wheel_signal)
  {
    return get_signals_by_joystick_coords(&jst, joystick_x, joystick_y,
				   left_wheel_signal, right_wheel_signal);
  }


  // Получить положение джойстика по значению сигналов колёс робота.
  int u2j(int left_wheel_signal, int right_wheel_signal,
          int *joystick_x, int *joystick_y)
  {
    return get_joystick_coords_by_signals(&jst, left_wheel_signal, right_wheel_signal,
				   joystick_x, joystick_y);
  }

};

#endif
