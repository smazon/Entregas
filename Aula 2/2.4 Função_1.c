#include <stdio.h>

int functionMDC (int num1 , int num2);

int main()
{   int num1= 17, num2= 31, i, hcf;
    
    printf("Numero 1: %d \n", num1);
    
    printf("Numero 2: %d  \n", num2);
    
    hcf = functionMDC(num1,num2);
    printf ("H.C.F of %d and %d is %d \n" , num1, num2, hcf);
    return 0;
}

int functionMDC (int num1 , int num2){
    int i, hcf;
    for (i=1; i<=num1 || i<=num2; ++i){
        if (num1%i==0 && num2%i==0)
            hcf=i;
    }
    return hcf;
}
