// MVM
// SOLVE FOR CONTROL INPUT TO MATH PATH ....
// 2D SISO system  example
#include <stdio.h>
#include <iostream>

// FOR matrix
#include <vector>
#include <Eigen/Dense>



// SYSTEM
#define S_DIM 2
#define U_DIM 1
#define O_DIM 1

//OVERALL
#define TOT_T 8
//#define TOT_T 20
//#define TOT_T 120


#define BIG_DIM  TOT_T


Eigen::VectorXd D(BIG_DIM); // data
Eigen::VectorXd u(BIG_DIM);  // Contol


Eigen::Vector2d x0;


Eigen::Vector2d y; // OUtput

double result[TOT_T+1];


Eigen::MatrixXd A(S_DIM, S_DIM );
Eigen::MatrixXd B(S_DIM,U_DIM );
Eigen::MatrixXd C(O_DIM, S_DIM );



void print_res(void)
{
	FILE* out_f;
	out_f=fopen("RES","w");
        int i;
        for(i=0;i<TOT_T;i++){
                fprintf(out_f,"%f\t%f\n",D(i),result[i]);
        }
	fclose(out_f);

}


void simulate(Eigen::VectorXd  u)
{
	int i;
	Eigen::Vector2d x;
	Eigen::VectorXd y(1);
	x=x0;
	y=C*x;
	for(i=0;i<TOT_T;i++){
		x=A*x+B*u(i);
		y=C*x;
		result[i]=y(0);	
		printf("%f (error =%f)\n",result[i],result[i]-D(i));
	}
	printf("\n");
}





// READ DATA INTO 
void read_data(void)
{
        FILE* o_file;
        int i;
        float val;
        o_file=fopen("PATH","r");
        for(i=0;i<TOT_T;i++){
                fscanf(o_file,"%f",&val);
                D(i)=val;
		fscanf(o_file,"%f",&val);
                //D(i)=val
        }
}





void init(void)
{
	int i,j;

	//A<<1,1,  0,.9;
	A<<1,1,0,1;
	B<<0,1;
	C<<1,0;
	x0<<0,0;

	std::cout<<A<<std::endl;
	std::cout<<"------------------"<<std::endl;

	Eigen::MatrixXd iA(S_DIM, S_DIM );
	Eigen::Vector2d  tmp;

	Eigen::VectorXd x=x0;

	double div = (C*B)(0); // SISO one OUTPUT
	// x(n) = A* x(n-1) + Bu(n-1) and y(n) = C x(n)
	// So y_n - y(n) = y_n - C (A x(n-1) - B u(n-1))
	// WE want zero error so
	//  u(n-1) =  y_n - C*A*x

	for(i=1;i<TOT_T;i++){  
		u(i-1)= ( D(i) -  (C*A*x)(0)) ;
		x=x+B*u(i-1);
		x=A*x;
	}

	std::cout<<"------------------"<<std::endl;
	std::cout<<u<<std::endl;
	std::cout<<"------------------"<<std::endl;
}




int main(void)
{
	read_data();
        std::cout<<"------------------"<<std::endl;
	std::cout<<D<<std::endl;
        std::cout<<"------------------"<<std::endl;

	init();
	simulate(u);
	print_res();
}