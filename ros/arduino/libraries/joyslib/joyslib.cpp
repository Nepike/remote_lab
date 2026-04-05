/***

***/

// расчёт сигнала, который надо подавать на колёса,
// по положению джойстика.

#include <math.h>
#include <stdlib.h>
#include "joyslib.h"

// Задать параметры преобразования сигналов джойстика в сигналы колёс и обратно.
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
