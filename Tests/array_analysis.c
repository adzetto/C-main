#include <stdio.h>
#include <stdlib.h>
#define N 10
#include <time.h>

void fillRand(int arr[100], int mod); // Fills the given array with random numbers who are in the range of 0-mod. mod variable can be change in the main function.
void fillArrZero(int arr[100]); // Sets all the values of array with zero.
void printArr(int arr[100]); // Prints the given function. ex -> (index) = (value)\n
int minArr(int arr[100]); // Finds the minimum value of a given array
void frequency(int arr[100], int arr2[100]); // Calculates the repetetion of a value of a given arr[100] and stores the repetition in an arr2[100].

int main() {
    int randNums[100], freq[100];
    int mod = 20; // you can change but be carefull about the randNums and freq arrays' length.
    fillRand(randNums, mod);
    fillArrZero(freq);
    frequency(randNums, freq);
    // visualisation begins
    printf("\n\n\n***********************");
    printf("\n\n ------randNums------\n");
    printf("\n***********************");
    // visualisation end
    printArr(randNums);
    printf("\n\n\n***********************");
    printf("\n\n ------Frequency------\n");
    printf("\n***********************");
    printArr(freq);
    printf("\n\n\n***********************");
    printf("\n\n ------Histogram------\n");
    printf("\n***********************");
    printHistogram(freq);

    // waits the input in order to see the output of the main.exe.
    int wait;
    getch(wait);


    return 0;
}


void frequency(int arr[100], int arr2[100]) {
    int i, j, minimum=minArr(arr);
    for(i=0;i<100;i++) {
        for(j=0;j<100;j++) {
            if(arr[j] == minimum) {
                arr2[i]++; 
            }
        }
        minimum++;
    }
}

int minArr(int arr[100]) {
    int i, min=arr[0];
    for(i=0;i<100;i++) {
        if(arr[i] < min) min = arr[i];
    }

    return min;
}

void printArr(int arr[100]) {
    int i;
    for(i=0;i<100;i++) {
        if(i < 10) {
            printf("\n %d. = %d", i, arr[i]);    
        }
        else {
            printf("\n%d. = %d", i, arr[i]);
        }
    }
}

void printHistogram(int arr[100]) {
    int i, j;
    for(i=0;i<100;i++) {
        if(i< 10){
            printf("\n %d: ", i);
            for(j=0;j<arr[i];j++) {
                printf("* ");
            }    
        }
        else{
            printf("\n%d: ", i);
            for(j=0;j<arr[i];j++) {
                printf("* ");
            }
        }
    }
}

void fillArrZero(int arr[100]) {
    int i;
    for(i=0;i<100;i++) {
        arr[i] = 0;
    }
}

void fillRand(int arr[100], int mod) {
    int i;

    time_t t; 
	srand((unsigned)time(&t));

    for(i=0;i<100;i++) {
        arr[i] = rand()%mod;
    }
}