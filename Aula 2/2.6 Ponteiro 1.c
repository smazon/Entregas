#include <stdio.h>

int main()

{
    int count=10, *temp, sum=0;
    temp = &count; // temp aponta para o endereco de memoria do count
    *temp = 20; // o ponteiro temp recebe o valor 20, logo count tambem e igual a 20
    temp = &sum; // temp aponta para o endereco de memoria do sum
    *temp = count; // o ponteiro temp recebe o valor de count (20), logo sum tambem e igual a 20
    printf("count= %d, *temp = %d, sum =%d\n", count, *temp, sum);
    return 0;
}// portanto ao rodar o codigo a saida deve ser count=20, temp= 20 e sum =20 

