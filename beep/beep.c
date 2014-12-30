#include <stdio.h>
#include <windows.h>
#include <stdlib.h>


void beep(){
	int count = 0;
	int intervalo, tempo;
	char resposta;
	
	printf("Digite o intervalo de tempo dos beeps: ");
	scanf("%d", &intervalo);
	printf("Digite o tempo total em segundos: ");
	scanf("%d", &tempo);
	
	while(count != tempo/intervalo){
		printf("%d\n", count*intervalo);
		printf("\a");
		count++;
		Sleep(intervalo*1000);
	}
	//printf("%d\n", (count++)*intervalo);
	//printf("\a");
	
	printf("Deseja executar novamente (s ou n)? ");
	scanf("%s", &resposta);
	if(resposta == 's')
		beep();
	else exit(EXIT_SUCCESS);
}

void main(){
	
	beep();
		
}