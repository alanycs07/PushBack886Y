#ifndef _FUNCTIONS_HPP
#define _FUNCTIONS_HPP

void drivePID(int distance,int time=5000);
// void drivePID(int distance);
void turnPID (int degrees, int time = 5000);
int limiter(int input);








#endif 