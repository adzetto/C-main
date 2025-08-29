#include <stdio.h>
#include <stdlib.h>

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

int main() {
	
	int Numbers[120],primes[100];
	int i,j,k=0,temp;
	
	for(i=2;i<121;i++){
		Numbers[i]=i;
	}
	
	for(i=2;i<12;i++){
		for(j=2;j<121;j++){
			if(j>i && Numbers[j]!=0){
			temp=Numbers[j];
			Numbers[j]=Numbers[j]%i;
			if(Numbers[j]!=0){
				Numbers[j]=temp;
			}
	    	}		
		}
	}
	
	for(i=2;i<121;i++){
		if(Numbers[i]!=0){
			primes[k]=Numbers[i];
			k++;
		}	
	}
    
	for(i=0;i<k-1;i++){
		printf("%d ",primes[i]);
	}
	
	
	
	
	
	
}
