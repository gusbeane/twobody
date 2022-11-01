#include <math.h>

extern "C" {
	void twobody(
	        const double *P0Pos_in, // Read-Only Vector 1
	        const double *P0Vel_in, // Read-Only Vector 2
	        const double *P1Pos_in, // Read-Only Vector 1
	        const double *P1Vel_in, // Read-Only Vector 2
	        double *P0Pos_out,       // Output Result
	        double *P0Vel_out,       // Output Result
	        double *P1Pos_out,       // Output Result
	        double *P1Vel_out,
		const double G,
		const double M0,
		const double M1,
                const double dt,
	 	const unsigned int N 
	        )
	{
#pragma HLS INTERFACE m_axi port=P0Pos_in bundle=aximm1
#pragma HLS INTERFACE m_axi port=P0Vel_in bundle=aximm1
#pragma HLS INTERFACE m_axi port=P1Pos_in bundle=aximm2
#pragma HLS INTERFACE m_axi port=P1Vel_in bundle=aximm2
#pragma HLS INTERFACE m_axi port=P0Pos_out bundle=aximm1
#pragma HLS INTERFACE m_axi port=P0Vel_out bundle=aximm1
#pragma HLS INTERFACE m_axi port=P1Pos_out bundle=aximm2
#pragma HLS INTERFACE m_axi port=P1Vel_out bundle=aximm2
#pragma HLS INTERFACE m_axi port=G bundle=aximm1
#pragma HLS INTERFACE m_axi port=M0 bundle=aximm1
#pragma HLS INTERFACE m_axi port=M1 bundle=aximm1
#pragma HLS INTERFACE m_axi port=dt bundle=aximm1
#pragma HLS INTERFACE m_axi port=N bundle=aximm1

            double P0Pos[3], P0Vel[3], P1Pos[3], P1Vel[3];
            double dPos[3], Acc[3];
            double dr, dr3;
            int i;
            double dt2 = dt/2.0;

            for(i=0 ; i<3; i++)
            {
                P0Pos[i] = P0Pos_in[i];
                P0Vel[i] = P0Vel_in[i];
                P1Pos[i] = P1Pos_in[i];
                P1Vel[i] = P1Vel_in[i];
            }

            // Do first aceleration step
            dPos[0] = P1Pos[0] - P0Pos[0];
            dPos[1] = P1Pos[1] - P0Pos[1];
            dPos[2] = P1Pos[2] - P0Pos[2];
	    
            dr = sqrt(dPos[0]*dPos[0] + dPos[1]*dPos[1] + dPos[2]*dPos[2]);
            dr3 = pow(dr, 3.);

            // Compute acceleration on P0
            Acc[0] = G * M1 * dPos[0] / dr3;
            Acc[1] = G * M1 * dPos[1] / dr3;
            Acc[2] = G * M1 * dPos[2] / dr3;

	    for(i = 0; i < N; i++)
            {
                // Do half-kick 
                P0Vel[0] += Acc[0] * dt2;
                P0Vel[1] += Acc[1] * dt2;
                P0Vel[2] += Acc[2] * dt2;
                
                P1Vel[0] += -(M0/M1) * Acc[0] * dt2;
                P1Vel[1] += -(M0/M1) * Acc[1] * dt2;
                P1Vel[2] += -(M0/M1) * Acc[2] * dt2;

                // Do drift
                P0Pos[0] += P0Vel[0] * dt;
                P0Pos[1] += P0Vel[1] * dt;
                P0Pos[2] += P0Vel[2] * dt;
                
                P1Pos[0] += P1Vel[0] * dt;
                P1Pos[1] += P1Vel[1] * dt;
                P1Pos[2] += P1Vel[2] * dt;
        
                // Compute forces
                dPos[0] = P1Pos[0] - P0Pos[0];
                dPos[1] = P1Pos[1] - P0Pos[1];
                dPos[2] = P1Pos[2] - P0Pos[2];
	    
                dr = sqrt(dPos[0]*dPos[0] + dPos[1]*dPos[1] + dPos[2]*dPos[2]);
                dr3 = pow(dr, 3.);

                // Compute acceleration on P0
                Acc[0] = G * M1 * dPos[0] / dr3;
                Acc[1] = G * M1 * dPos[1] / dr3;
                Acc[2] = G * M1 * dPos[2] / dr3;

                // Do half-kick 
                P0Vel[0] += Acc[0] * dt2;
                P0Vel[1] += Acc[1] * dt2;
                P0Vel[2] += Acc[2] * dt2;
                
                P1Vel[0] += -(M0/M1) * Acc[0] * dt2;
                P1Vel[1] += -(M0/M1) * Acc[1] * dt2;
                P1Vel[2] += -(M0/M1) * Acc[2] * dt2;
            }

            for(i=0; i<3; i++){
                P0Pos_out[i] = P0Pos[i];
                P0Vel_out[i] = P0Vel[i];
                P1Pos_out[i] = P1Pos[i];
                P1Vel_out[i] = P1Vel[i];
            }
	}
}

