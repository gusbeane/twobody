/**********
Copyright (c) 2019, Xilinx, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********/


#include "event_timer.hpp"

#include <iostream>
#include <memory>
#include <string>

// Xilinx OpenCL and XRT includes
#include "xilinx_ocl_helper.hpp"
#include <math.h>

void twobody_sw(const double *P0Pos_in, // Read-Only Vector 1
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

int main(int argc, char *argv[])
{
    // Initialize an event timer we'll use for monitoring the application
    EventTimer et;

    std::cout << "-- Example 3: Allocate and Map Contiguous Buffers --" << std::endl
              << std::endl;

    // Initialize the runtime (including a command queue) and load the
    // FPGA image
    std::cout << "Loading alveo_examples.xclbin to program the Alveo board" << std::endl
              << std::endl;
    et.add("OpenCL Initialization");

    // This application will use the first Xilinx device found in the system
    xilinx::example_utils::XilinxOclHelper xocl;
    xocl.initialize("twobody.xclbin");

    cl::CommandQueue q = xocl.get_command_queue();
    cl::Kernel krnl    = xocl.get_kernel("twobody");
    et.finish();

    /// New code for example 01
    std::cout << "Running kernel test with XRT-allocated contiguous buffers" << std::endl;


    double M0, M1, G, dt;
    unsigned int N;
    M0 = 1.0;
    M1 = 3.003E-6;
    G = 1.0;
    dt = 2*3.1416/365.0;
    N = 365*10000;
    // Map our user-allocated buffers as OpenCL buffers using a shared
    // host pointer
    et.add("Allocate contiguous OpenCL buffers");
    cl::Buffer P0Pos_in_buf(xocl.get_context(),
                     static_cast<cl_mem_flags>(CL_MEM_READ_ONLY |
                                               CL_MEM_ALLOC_HOST_PTR),
                     3 * sizeof(double),
                     NULL,
                     NULL);
    cl::Buffer P0Vel_in_buf(xocl.get_context(),
                     static_cast<cl_mem_flags>(CL_MEM_READ_ONLY |
                                               CL_MEM_ALLOC_HOST_PTR),
                     3 * sizeof(double),
                     NULL,
                     NULL);
    cl::Buffer P1Pos_in_buf(xocl.get_context(),
                     static_cast<cl_mem_flags>(CL_MEM_READ_ONLY |
                                               CL_MEM_ALLOC_HOST_PTR),
                     3 * sizeof(double),
                     NULL,
                     NULL);
    cl::Buffer P1Vel_in_buf(xocl.get_context(),
                     static_cast<cl_mem_flags>(CL_MEM_READ_ONLY |
                                               CL_MEM_ALLOC_HOST_PTR),
                     3 * sizeof(double),
                     NULL,
                     NULL);
    cl::Buffer P0Pos_out_buf(xocl.get_context(),
                     static_cast<cl_mem_flags>(CL_MEM_WRITE_ONLY |
                                               CL_MEM_ALLOC_HOST_PTR),
                     3 * sizeof(double),
                     NULL,
                     NULL);
    cl::Buffer P0Vel_out_buf(xocl.get_context(),
                     static_cast<cl_mem_flags>(CL_MEM_WRITE_ONLY |
                                               CL_MEM_ALLOC_HOST_PTR),
                     3 * sizeof(double),
                     NULL,
                     NULL);
    cl::Buffer P1Pos_out_buf(xocl.get_context(),
                     static_cast<cl_mem_flags>(CL_MEM_WRITE_ONLY |
                                               CL_MEM_ALLOC_HOST_PTR),
                     3 * sizeof(double),
                     NULL,
                     NULL);
    cl::Buffer P1Vel_out_buf(xocl.get_context(),
                     static_cast<cl_mem_flags>(CL_MEM_WRITE_ONLY |
                                               CL_MEM_ALLOC_HOST_PTR),
                     3 * sizeof(double),
                     NULL,
                     NULL);

    et.finish();

    // Set vadd kernel arguments
    et.add("Set kernel arguments");
    krnl.setArg(0, P0Pos_in_buf);
    krnl.setArg(1, P0Vel_in_buf);
    krnl.setArg(2, P1Pos_in_buf);
    krnl.setArg(3, P1Vel_in_buf);
    krnl.setArg(4, P0Pos_out_buf);
    krnl.setArg(5, P0Vel_out_buf);
    krnl.setArg(6, P1Pos_out_buf);
    krnl.setArg(7, P1Vel_out_buf);
    krnl.setArg(8, G);
    krnl.setArg(9, M0);
    krnl.setArg(10, M1);
    krnl.setArg(11, dt);
    krnl.setArg(12, N);

    et.add("Map buffers to userspace pointers");
    double *P0Pos_in = (double *)q.enqueueMapBuffer(P0Pos_in_buf,
                                                 CL_TRUE,
                                                 CL_MAP_WRITE,
                                                 0,
                                                 3 * sizeof(double));
    double *P0Vel_in = (double *)q.enqueueMapBuffer(P0Vel_in_buf,
                                                 CL_TRUE,
                                                 CL_MAP_WRITE,
                                                 0,
                                                 3 * sizeof(double));
    double *P1Pos_in = (double *)q.enqueueMapBuffer(P1Pos_in_buf,
                                                 CL_TRUE,
                                                 CL_MAP_WRITE,
                                                 0,
                                                 3 * sizeof(double));
    double *P1Vel_in = (double *)q.enqueueMapBuffer(P1Vel_in_buf,
                                                 CL_TRUE,
                                                 CL_MAP_WRITE,
                                                 0,
                                                 3 * sizeof(double));
    et.finish();

    et.add("Populating buffer inputs");
    P0Pos_in[0] = P0Pos_in[1] = P0Pos_in[2] = 0.0;
    P0Vel_in[0] = P0Vel_in[1] = P0Vel_in[2] = 0.0;
    P1Pos_in[0] = 1.0;
    P1Pos_in[1] = P1Pos_in[2] = 0.0;
    P1Vel_in[0] = P1Vel_in[2] = 0.0;
    P1Vel_in[1] = sqrt(G * M0 / 1.0);

    double COMPos[3], COMVel[3];
    for(int i=0; i<3; i++)
    {
        COMPos[i] = (M0 * P0Pos_in[i] + M1 * P1Pos_in[i]) / (M0 + M1);
        P0Pos_in[i] -= COMPos[i];
        P1Pos_in[i] -= COMPos[i];
        
        COMVel[i] = (M0 * P0Vel_in[i] + M1 * P1Vel_in[i]) / (M0 + M1);
        P0Vel_in[i] -= COMVel[i];
        P1Vel_in[i] -= COMVel[i];
    }


    et.finish();

    double P0Pos_out_sw[3], P0Vel_out_sw[3], P1Pos_out_sw[3], P1Vel_out_sw[3];

    // For comparison, let's have the CPU calculate the result
    et.add("Software twobody run");
    twobody_sw(P0Pos_in, P0Vel_in, P1Pos_in, P1Vel_in,
               P0Pos_out_sw, P0Vel_out_sw, P1Pos_out_sw, P1Vel_out_sw,
               G, M0, M1, dt, N);
    et.finish();

    // Send the buffers down to the Alveo card
    et.add("Memory object migration enqueue");
    cl::Event event_sp;
    q.enqueueMigrateMemObjects({P0Pos_in_buf, P0Vel_in_buf, P1Pos_in_buf, P1Vel_in_buf}, 0, NULL, &event_sp);
    clWaitForEvents(1, (const cl_event *)&event_sp);

    et.add("OCL Enqueue task");

    q.enqueueTask(krnl, NULL, &event_sp);
    et.add("Wait for kernel to complete");
    clWaitForEvents(1, (const cl_event *)&event_sp);

    // Migrate memory back from device
    et.add("Read back computation results");
    double *P0Pos_out_hw = (double *)q.enqueueMapBuffer(P0Pos_out_buf,
                                                 CL_TRUE,
                                                 CL_MAP_READ,
                                                 0,
                                                 3 * sizeof(double));
    double *P0Vel_out_hw = (double *)q.enqueueMapBuffer(P0Vel_out_buf,
                                                 CL_TRUE,
                                                 CL_MAP_READ,
                                                 0,
                                                 3 * sizeof(double));
    double *P1Pos_out_hw = (double *)q.enqueueMapBuffer(P1Pos_out_buf,
                                                 CL_TRUE,
                                                 CL_MAP_READ,
                                                 0,
                                                 3 * sizeof(double));
    double *P1Vel_out_hw = (double *)q.enqueueMapBuffer(P1Vel_out_buf,
                                                 CL_TRUE,
                                                 CL_MAP_READ,
                                                 0,
                                                 3 * sizeof(double));
    et.finish();



    // Verify the results
    bool verified = true;
    for (int i = 0; i < 3; i++) {
        if (P1Pos_out_hw[i] != P1Pos_out_sw[i]) {
            verified = false;
            std::cout << "ERROR: software and hardware vadd do not match: "
                      << P1Pos_out_hw[i] << "!=" << P1Pos_out_sw[i] << " at position " << i << std::endl;
            break;
        }
    }

    if (verified) {
        std::cout
            << std::endl
            << "OCL-mapped contiguous buffer example complete!"
            << std::endl
            << std::endl;
    }
    else {
        std::cout
            << std::endl
            << "OCL-mapped contiguous buffer example complete! (with errors)"
            << std::endl
            << std::endl;
    }

    std::cout << "--------------- Key execution times ---------------" << std::endl;


    q.enqueueUnmapMemObject(P0Pos_in_buf, P0Pos_in);
    q.enqueueUnmapMemObject(P0Vel_in_buf, P0Vel_in);
    q.enqueueUnmapMemObject(P1Pos_in_buf, P1Pos_in);
    q.enqueueUnmapMemObject(P1Vel_in_buf, P1Vel_in);
    q.enqueueUnmapMemObject(P0Pos_out_buf, P0Pos_out_hw);
    q.enqueueUnmapMemObject(P0Vel_out_buf, P0Vel_out_hw);
    q.enqueueUnmapMemObject(P1Pos_out_buf, P1Pos_out_hw);
    q.enqueueUnmapMemObject(P1Vel_out_buf, P1Vel_out_hw);
    q.finish();


    et.print();
}
