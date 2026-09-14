// MVM
// SOLVE FOR CONTROL INPUT TO MATH PATH ....
// 2D SISO system  example 123123
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


// BIG ONE s
Eigen::MatrixXd G(BIG_DIM,TOT_T); 
Eigen::VectorXd H(BIG_DIM);



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
	// Data  is special !!
	//fscanf(o_file,"%f",&val);
	///fscanf(o_file,"%f",&val);
        for(i=0;i<TOT_T;i++){
                fscanf(o_file,"%f",&val);
                D(i)=val;
		fscanf(o_file,"%f",&val);
                //data[i][1]=val;
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

	// BIG MATRIX Eigen::MatrixXd G(BIG_DIM,BIG_DIM);
	for(i=0;i<TOT_T;i++){  
		iA=A;
		tmp=B;
		for(j=0;j<=i;j++){ //  LOWER DIAGRONAL+
			G(i,i-j)=   (C*tmp)(0); // SISO
			tmp=iA*B;
	   		iA=iA*A;
		}

	}
	std::cout<<G<<std::endl;
 	/// ALSO COMPUTE INTERNAL STATE DYNAMIC FROM X0 (THis is usually 0 ...)
	tmp=x0;
	for(i=0;i<TOT_T;i++){
		H(i)  = (C*tmp)(0);//  SISO
		tmp=A*tmp;
	}

	std::cout<<"------------------"<<std::endl;
	std::cout<<H<<std::endl;
	std::cout<<"------------------"<<std::endl;
}


void solve_it(void)
{
	
	Eigen::VectorXd sol(TOT_T);
	sol  = G.colPivHouseholderQr().solve(D);
	std::cout<<"------- SOLUTION u -----------"<<std::endl;
	std::cout<<sol<<std::endl;
	std::cout<<"-------  SIMULATION -----------"<<std::endl;

	simulate(sol);
}




int main(void)
{
	read_data();
        std::cout<<"------------------"<<std::endl;
	std::cout<<D<<std::endl;
        std::cout<<"------------------"<<std::endl;

	init();
	solve_it();
	print_res();
}