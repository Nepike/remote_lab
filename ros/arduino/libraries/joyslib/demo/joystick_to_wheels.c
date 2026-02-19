// расчёт сигнала, который надо подавать на колёса,
// по положению джойстика.
#include<stdio.h>
#include<math.h>
#include<assert.h>

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
					    int max_wheel_signal){
  // если в параметрах ошибка, возвращаем -1
  if(min_joystick_x >= max_joystick_x
     || min_joystick_y >= max_joystick_y
     || min_wheel_signal >= max_wheel_signal)
    return -1;
  // если структура не задана, возвращаем -2
  if(p==NULL)
    return -2;
  // задаём параметры
  p->middle_joystick_x = (max_joystick_x + min_joystick_x)/2.0;
  p->middle_joystick_y = (max_joystick_y + min_joystick_y)/2.0;
  p->middle_wheel_signal = (max_wheel_signal + min_wheel_signal)/2.0;
  p->amplitude_joystick_x = (max_joystick_x - min_joystick_x)/2.0;
  p->amplitude_joystick_y = (max_joystick_y - min_joystick_y)/2.0;
  p->amplitude_wheel_signal = (max_wheel_signal - min_wheel_signal)/2.0;
  return 0;
}

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
				   int *left_wheel_signal, int *right_wheel_signal){
  // если переданы неверные хранилища результатов или параметры
  if(right_wheel_signal==NULL 
     || left_wheel_signal==NULL
     || p==NULL)
    return -2;
  // отклонения координат джойстика от центрального значения
  float dx = joystick_x - p->middle_joystick_x;
  float dy = joystick_y - p->middle_joystick_y;
  // если на входе недопустимое значение, выходим с ошибкой
  if(dx > p->amplitude_joystick_x
     || dx < - p->amplitude_joystick_x
     || dy > p->amplitude_joystick_y
     || dy < - p->amplitude_joystick_y)
    return -1;
  // приводим сигналы к диапазону [-1, 1]
  float fx = dx / p->amplitude_joystick_x;
  float fy = dy / p->amplitude_joystick_y;
  // если сигнаны в этом диапазоне, но их сочетание вышло 
  // за пределы единичного круга, проецируем сигналы в 
  // точку на границе круга
  if(fx*fx+fy*fy>1){
    float len = sqrt(fx*fx + fy*fy);
    fx /= len;
    fy /= len;
  }
  // значения скоростей
  float sq = sqrt(fx*fx + fy*fy);
  int v1 = floor(p->amplitude_wheel_signal*sq + 0.5);
  int v2 = floor(p->amplitude_wheel_signal*sq*(fx*fx - fy*fy));
  // угол, в зависимости от которого определяются координаты
  float an = atan2(fy, fx);
  // варианты в зависимости от угла
  if(an<=-M_PI/2 || an>=M_PI){ // PI --- полный назад.
    *left_wheel_signal = -v1;
    *right_wheel_signal = v2;
  }else if(an<=0){
    *left_wheel_signal = v2;
    *right_wheel_signal = -v1;
  }else if(an<=M_PI/2){
    *left_wheel_signal = v1;
    *right_wheel_signal = -v2;
  }else{
    *left_wheel_signal = -v2;
    *right_wheel_signal = v1;
  }
  return 0;
}

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
				   int *joystick_x, int *joystick_y){
  // если переданы неверные указатели, возвращаем -2.
  if(p==NULL
     || joystick_x==NULL
     || joystick_y==NULL)
    return -2;
  // если входные значения не входят в допустимый диапазон, возвращаем -1
  float dl = left_wheel_signal - p->middle_wheel_signal;
  float dr = right_wheel_signal - p->middle_wheel_signal;
  if(dl > p->amplitude_wheel_signal
     || dl < -p->amplitude_wheel_signal
     || dr > p->amplitude_wheel_signal
     || dr < -p->amplitude_wheel_signal)
    return -1;
  // приводим входные значения к диапазону [-1, 1]
  float fl = dl/p->amplitude_wheel_signal;
  float fr = dr/p->amplitude_wheel_signal;
  float an = M_PI/4 + atan2(fr, fl);
  float tmp_x, tmp_y;
  int sgn = 1;
  if(an<-M_PI/2){
    tmp_x = sqrt((fl*fl-fl*fr)/2);
    tmp_y = sqrt((fl*fl+fl*fr)/2);
    sgn = (fl>0)?1:-1;
  }else if(an<0){
    tmp_x = -sqrt((fr*fr-fl*fr)/2);
    tmp_y = sqrt((fr*fr+fl*fr)/2);
    sgn = (fr>0)?1:-1;
  }else if(an<M_PI/2){
    tmp_x = sqrt((fl*fl-fl*fr)/2);
    tmp_y = sqrt((fl*fl+fl*fr)/2);
    sgn = (fl>0)?1:-1;
  }else{
    tmp_x = -sqrt((fr*fr-fl*fr)/2);
    tmp_y = sqrt((fr*fr+fl*fr)/2);
    sgn = (fr>0)?1:-1;
  }
  *joystick_x = floor(sgn*tmp_x*p->amplitude_joystick_x + p->middle_joystick_x + 0.5);
  *joystick_y = floor(sgn*tmp_y*p->amplitude_joystick_y + p->middle_joystick_y + 0.5);
  return 0;
}

// встроенный тест + пример использования
// функции set_joystick_to_wheels_transform_params
void test_set_joystick_to_wheels_transform_params(){
  int res;
  const float eps = 1e-6; // точность сравнения результатов с эталоном
  // Тест 1. Создаём структуру с параметрами преобразования
  struct joystick_to_wheels_transform_params jw_params;
  res = set_joystick_to_wheels_transform_params(&jw_params,
						-100, 200,
						-200, 100,
						0, 101);
  // сравниваем полученные значения с эталонными
  assert(res==0);
  assert(fabs(jw_params.middle_joystick_x - 50)<eps);
  assert(fabs(jw_params.middle_joystick_y + 50)<eps);
  assert(fabs(jw_params.middle_wheel_signal - 50.5)<eps);
  assert(fabs(jw_params.amplitude_joystick_x - 150)<eps);
  assert(fabs(jw_params.amplitude_joystick_y - 150)<eps);
  assert(fabs(jw_params.amplitude_wheel_signal - 50.5)<eps);
  // Тест 2. То же, но с неверным указателем на структуру хранения
  assert(-2 == set_joystick_to_wheels_transform_params(NULL,
						       -100, 200,
						       -200, 100,
						       0, 101));
  // Тест 3. Как 1, но один из диапазонов нулевой
  assert(-1 == set_joystick_to_wheels_transform_params(&jw_params,
						       -100, 100,
						       -200, 100,
						       11, 11));
  // Тест 4. Как 1, но один из максимумов меньше другого
  assert(-1==set_joystick_to_wheels_transform_params(&jw_params,
						     -100, 100,
						     200, -100,
						     0, 101));
}


// встроенный тест для get_signals_by_joystick_coords
void test_get_signals_by_joystick_coords(){
  int res;
  int lw = 0, rw = 0;
  const float eps = 1e-6;
  // создаём структуру с параметрами преобразования
  struct joystick_to_wheels_transform_params jw_params;
  res = set_joystick_to_wheels_transform_params(&jw_params,
						-100, 100,
						-100, 100,
						-100, 100);
  // Проверки ошибок в данных
  // Тест 1. Нулевой указатель на описание преобразований
  assert(-2 == get_signals_by_joystick_coords(NULL,
					      0, 0,
					      &lw, &rw));
  // Тест 2. Нулевой указатель на данные
  assert(-2 == get_signals_by_joystick_coords(&jw_params,
					      0, 0,
					      &lw, NULL));
  // Тест 3. X за пределами допустимого диапазона
  assert(-1 == get_signals_by_joystick_coords(&jw_params,
					      101, 0,
					      &lw, &rw));
  // Тест 4. Y за пределами допустимого диапазона
  assert(-1 == get_signals_by_joystick_coords(&jw_params,
					      10, -101,
					      &lw, &rw));
  // Проверки преобразований
  // Тест 5. Стоп.
  res = get_signals_by_joystick_coords(&jw_params,
				       0, 0,
				       &lw, &rw);
  assert(res==0);
  assert(fabs(lw)<eps);
  assert(fabs(rw)<eps);
  // Тест 6. Полный вперёд.
  res = get_signals_by_joystick_coords(&jw_params,
				       0, 100,
				       &lw, &rw);
  assert(res==0);
  assert(fabs(lw-100)<eps);
  assert(fabs(rw-100)<eps);
  // Тест 7. Полный назад.
  res = get_signals_by_joystick_coords(&jw_params,
				       0, -100,
				       &lw, &rw);
  assert(res==0);
  assert(fabs(lw+100)<eps);
  assert(fabs(rw+100)<eps);
  // Тест 8. На месте налево.
  res = get_signals_by_joystick_coords(&jw_params,
				       -100, 0,
				       &lw, &rw);
  assert(res==0);
  assert(fabs(lw+100)<eps);
  assert(fabs(rw-100)<eps);
  // Тест 9. На месте направо.
  res = get_signals_by_joystick_coords(&jw_params,
				       100, 0,
				       &lw, &rw);
  assert(res==0);
  assert(fabs(lw-100)<eps);
  assert(fabs(rw+100)<eps);
  // Тест 10. Каждый из сигналов попадает в диапазон, но
  // точка, заданная ими, не попадает в допустимый круг
  lw = -1;
  rw = -1;
  res = get_signals_by_joystick_coords(&jw_params,
				       99, 99,
				       &lw, &rw);
  assert(res==0);
  assert(fabs(lw)<=100);
  assert(fabs(rw)<=100);
}

// встроенный тест get_joystick_coords_by_signals
void test_get_joystick_coords_by_signals(){
  int res;
  int jx = 0, jy = 0;
  const float eps = 1e-6;
  // создаём структуру с параметрами преобразования
  struct joystick_to_wheels_transform_params jw_params;
  res = set_joystick_to_wheels_transform_params(&jw_params,
						-100, 100,
						-100, 100,
						-100, 100);
  // Тесты параметров
  // Тест 1. Нулевой указатель на описание преобразований
  assert(-2 == get_joystick_coords_by_signals(NULL,
					      0, 0,
					      &jx, &jy));
  // Тест 2. Нулевой указатель на данные
  assert(-2 == get_joystick_coords_by_signals(&jw_params,
					      0, 0,
					      &jx, NULL));
  // Тест 3. Левая скорость за пределами допустимого диапазона
  assert(-1 == get_joystick_coords_by_signals(&jw_params,
					      101, 0,
					      &jx, &jy));
  // Тест 4. Правая скорость за пределами допустимого диапазона
  assert(-1 == get_joystick_coords_by_signals(&jw_params,
					      10, -101,
					      &jx, &jy));
  // Проверки преобразований
  // Тест 5. Стоп.
  res = get_joystick_coords_by_signals(&jw_params,
				       0, 0,
				       &jx, &jy);
  assert(res==0);
  assert(fabs(jx)<eps);
  assert(fabs(jy)<eps);
  // Тест 6. Полный вперёд.
  res = get_joystick_coords_by_signals(&jw_params,
				       100, 100,
				       &jx, &jy);
  assert(res==0);
  assert(fabs(jx)<eps);
  assert(fabs(jy-100)<eps);
  // Тест 7. Полный назад.
  res = get_joystick_coords_by_signals(&jw_params,
				       -100, -100,
				       &jx, &jy);
  assert(res==0);
  assert(fabs(jx)<eps);
  assert(fabs(jy+100)<eps);
  // Тест 8. На месте налево.
  res = get_joystick_coords_by_signals(&jw_params,
				       -100, 100,
				       &jx, &jy);
  assert(res==0);
  assert(fabs(jx+100)<eps);
  assert(fabs(jy)<eps);
  // Тест 9. На месте направо.
  res = get_joystick_coords_by_signals(&jw_params,
				       100, -100,
				       &jx, &jy);
  assert(res==0);
  assert(fabs(jx-100)<eps);
  assert(fabs(jy)<eps);
}

int main(int argc, char *argv[])
{
  // Запуск встроенных тестов
  test_set_joystick_to_wheels_transform_params();
  test_get_signals_by_joystick_coords();
  test_get_joystick_coords_by_signals();
  // Если выполнение дошло до этого места, тесты выполнились успешно.
  printf("Test results are correct.\n");
  return 0;
}
