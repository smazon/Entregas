#include <stdio.h>

void print_array(int *p);

int main()
{   int i;
    int test_array[]={2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71};
    int *p;
    print_array(test_array);
    
    return 0;
}

void print_array(int *p){
    int i;
    for (i=0; i<20; i++){ 
        printf(" Primo %d = %d\n", i+1, *(p+i));
    }
}