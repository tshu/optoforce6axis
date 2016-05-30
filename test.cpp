/*
 * test.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Tony Shu
 *
 *	Detects the available Optoforce 6-axis transducer and opens USB communications in a streaming thread. Values are streamed for 5s
 *	in the format {period Fx Fy Fz Tx Ty Tz}. Streaming thread closes upon test completion and the USB port is restored to defaults.
 */
#include "optoDriver.hpp"

int main()
{
	optoDriver transducer(1, 4, true);
	if (transducer.getFD() > 0) {

		std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
		std::chrono::milliseconds elapsed;
		elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start);

		while (elapsed.count() < 5000){
			transducer.spinOnce();

				std::cout << "\n" << transducer.period.count() << "ms ";
				for (int i=0; i < transducer.forces_vec.size(); i++){
			    	printf("%3.2f ", transducer.forces_vec[i]);
			    	fflush(stdout);
			    }
			    

			elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start);
		}
		printf("\n");
	}
	return 1;

}


