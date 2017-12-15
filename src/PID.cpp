#include "PID.h"
#include <limits>
#include <math.h>
#include <vector>
#include <iostream>
#include <string>
using namespace std;
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
	PID::p_error = 0.0;
	PID::i_error = 0.0;
	PID::d_error = 0.0;
	PID::total_error = 0;
	PID::dp = {0.1,0.5,0.5};
	PID::Params={Kp,Ki,Kd};
	PID::iteration = 1;
	cout << "Initialized" << endl;
	PID::best_error = 100000.0;
	PID::loop_number=1;
	PID::index_param = 0; //which paramter to twiddle
	PID::temp_storage2={};
	
}

void PID::UpdateError(double Kp, double Ki, double Kd, double cte) {
	PID::d_error = cte - p_error;
	PID::p_error = cte;
	PID::i_error += cte;
	PID::Params={Kp,Ki,Kd};
	PID::total_error = PID::TotalError();
	if (PID::iteration == 1)
	{
		PID::best_error = PID::total_error;
		PID::Params[index_param] += PID::dp[index_param];
		cout << "loop 1" << endl;
		PID::prev_param_index=1;
	}
	else{
		if(PID::total_error <= PID::best_error) // improvement
		{
			cout << "loop 2" << endl;
			if(PID::loop_number == 2){
				PID::best_error = PID::total_error;
				PID::dp[index_param] *= 1.1;
				PID::loop_number =0;
				cout << "loop 3" << endl;
				PID::index_param +=1;
				if(PID::index_param == 4) // loop back to first param
				{
					cout << "loop 5" << endl;
					PID::index_param=0;
				}
				PID::Params[index_param] += PID::dp[index_param]; 
				
			}
			else {
				PID::best_error = PID::total_error;
				PID::dp[index_param] *=1.1;
				PID::index_param +=1; // move to next parameter twiddling
				cout << "loop 4" << endl;
				if(PID::index_param == 4) // loop back to first param
				{
					cout << "loop 5" << endl;
					PID::index_param=0;
				}
				PID::Params[index_param] += PID::dp[index_param]; 
			}
			
		}
		else{
			cout << "loop 6" << endl;
			
			if (PID::loop_number ==1) // if err found to be greater the first time
			{
				cout << "loop 7" << endl;
				PID::temp_storage = PID::Params[index_param];
				PID::Params[index_param] -= 2* PID::dp[index_param];
				PID::loop_number +=1;
				
			}
			if(PID::loop_number == 2)
			{
				cout << "loop 8" << endl;
				PID::Params[index_param] = PID::temp_storage;
				PID::dp[index_param] *=0.95;
				PID::Params[index_param] += PID::dp[index_param];
				PID::loop_number = 0;
				
			}
			
		}
	}
	iteration+=1;	
	if (iteration %4 ==0){
		if(PID::prev_param_index != PID::index_param){
			PID::prev_param_index = PID::index_param;
			PID::temp_storage2.clear();
		}
		else{
		PID::temp_storage2.push_back(PID::Params[index_param]);
		if (PID::temp_storage2.size() >4){
			PID::index_param +=1;
		}
		}
	}
	PID::Kp = PID::Params[0];
	PID::Ki = PID::Params[1];
	PID::Kd = PID::Params[2];
	cout << "Kp: " << PID::Params[0] << "   Kd: " << PID::Params[1] << "    Ki: " << PID::Params[2] << endl;
	cout << Kp+Kd+Ki << endl;
	
	
}


double PID::TotalError() {
	return -PID::Kp * PID::p_error - PID::Kd * PID::d_error - PID::Ki * PID::i_error;
}

