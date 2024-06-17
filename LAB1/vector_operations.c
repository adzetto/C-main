#include <stdio.h>

// Structure to represent a 3D vector
typedef struct {
    float x;
    float y;
    float z;
} Vector3D;

// Function to calculate the dot product of two 3D vectors
float dotProduct(Vector3D v1, Vector3D v2) {
    return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}

// Function to calculate the cross product of two 3D vectors
Vector3D crossProduct(Vector3D v1, Vector3D v2) {
    Vector3D result;
    result.x = (v1.y * v2.z) - (v1.z * v2.y);
    result.y = (v1.z * v2.x) - (v1.x * v2.z);
    result.z = (v1.x * v2.y) - (v1.y * v2.x);
    return result;
}

int main() {
    Vector3D vector1, vector2;

    // Taking input for vector1
    printf("Enter the components of the first vector (x y z): ");
    scanf("%f %f %f", &vector1.x, &vector1.y, &vector1.z);

    // Taking input for vector2
    printf("Enter the components of the second vector (x y z): ");
    scanf("%f %f %f", &vector2.x, &vector2.y, &vector2.z);

    // Calculate and print the dot product
    float dot = dotProduct(vector1, vector2);
    printf("Dot Product: %.2f\n", dot);

    // Calculate and print the cross product
    Vector3D cross = crossProduct(vector1, vector2);
    printf("Cross Product: %.2f, %.2f, %.2f\n", cross.x, cross.y, cross.z);

    return 0;
}
