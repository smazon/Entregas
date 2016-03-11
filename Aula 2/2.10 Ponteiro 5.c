#include <stdio.h>

int *primos(void);
int v[3];// a variavel deve ser setada como global para que o main e a funcao tenham acesso ao valor da memoria
int main(){
    primos();
    printf("%d,\n %d, \n %d,\n %d\n", &v, *(v),*(v+1),*(v+2));
}

int *primos(void){
    v[0]=1009;
	v[1]=1013;
    v[2]=1019;
    return v;
}