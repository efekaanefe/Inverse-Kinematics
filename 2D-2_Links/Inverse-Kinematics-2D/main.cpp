// https://www.alanzucconi.com/2018/05/02/ik-2d-1/
// https://www.youtube.com/watch?v=df5YwVsekmE&ab_channel=ZeroKelvinTutorials
#include "raylib.h"
#include "raymath.h"
#include <math.h>
#include <iostream>

# define PI  3.14159265358979323846  /* pi */

int widthScreen = 800;
int heightScreen = 800;

// coordinates
int x0Pos = widthScreen/2; 
int y0Pos = heightScreen / 2; // fixed joint coordinates
float jointRadius = 5;

// dimensions
float L1 = 150;
float theta1 = 60; // in degree
float L2 = 200;
float theta2 = 45;

float dTheta = 1;

float desiredTheta1 = 0;
float desiredTheta2 = 42;

// colors
Color jointColor = BLACK;
Color linkColor = DARKGRAY;
Color targetColor = RED;

bool isGoing = false;
bool targetIsSet = false;

Vector2 targetPos;


int main() {

	int currentFrame = 0;
	int framesToReach = 10;


	InitWindow(widthScreen, heightScreen, "Inverse-Kinematics-2D");
	SetTargetFPS(75);              

	while (!WindowShouldClose())
	{
		if (!targetIsSet) targetPos = GetMousePosition();
		
		if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) //and !targetIsSet)
		{
			
			// calculating a, b, c lengths of the triangle
			float c = L1;
			float a = L2;
			float b = sqrt(pow(x0Pos - targetPos.x, 2) + pow(y0Pos - targetPos.y, 2));

			if (b < L1 + L2) {
				float desiredAlpha = acos((pow(b, 2) + pow(c, 2) - pow(a, 2)) / (2 * b * c));
				float desiredBeta = acos((pow(a, 2) + pow(c, 2) - pow(b, 2)) / (2 * a * c));
				float Aprime = atan2((targetPos.y - y0Pos), (targetPos.x - x0Pos));
				float A = desiredAlpha - Aprime;
				float B = PI - desiredBeta;
				desiredTheta1 = (PI - A);
				desiredTheta2 = desiredTheta1 + B ;

				desiredTheta1 = desiredTheta1 * 180 / PI + 180;
				desiredTheta2 = desiredTheta2 * 180 / PI + 180;

				// to instantly change the angles
				/*theta1 = desiredTheta1;
				theta2 = desiredTheta2;*/

				targetIsSet = !targetIsSet; //target sets here
			}

		}

		if (IsKeyDown(KEY_SPACE) and targetIsSet) isGoing=true;


		BeginDrawing();
		{
			ClearBackground(RAYWHITE);

			DrawCircle(targetPos.x, targetPos.y, jointRadius, targetColor);


			DrawCircle(x0Pos, y0Pos, jointRadius+3, jointColor);

			// draw first link
			float x1Pos = x0Pos + L1 * cos(PI / 180 * theta1);
			float y1Pos = y0Pos + L1 * sin(PI / 180 * theta1);
			DrawLine(x0Pos, y0Pos, std::lround(x1Pos) , std::lround(y1Pos) , linkColor);

			DrawCircle(x1Pos, y1Pos, jointRadius, jointColor);

			// draw second link
			float x2Pos = x1Pos +  L2 * cos(PI / 180 * theta2);
			float y2Pos = y1Pos + L2 * sin(PI / 180 * theta2);
			DrawLine(x1Pos, y1Pos, std::lround(x2Pos), std::lround(y2Pos), linkColor);

			//debug
			//printf("theta1, theta2 | x2, y2 | target.x, target.y\n");
		/*	printf("%.2f , %.2f | ", theta1, theta2);
			printf("%.2f , %.2f | \n", desiredTheta1, desiredTheta2);*/

			//printf("%.2f , %.2f | ", x2Pos - x0Pos, y2Pos - y0Pos);
			//printf("%.2f , %.2f \n", targetPos.x - x0Pos, targetPos.y - y0Pos);

			if (targetIsSet and isGoing) {
				
				if (abs(theta1- desiredTheta1) > 1 or abs(theta2 - desiredTheta2) > 1) {
					theta1 = Lerp(theta1, desiredTheta1, static_cast<float>(currentFrame) / framesToReach);
					theta2 = Lerp(theta2, desiredTheta2, static_cast<float>(currentFrame) / framesToReach);

					currentFrame++;
				}
				else {
					currentFrame = 0;
					isGoing = !isGoing;
				}
			}
		}

		DrawText("MBL to fix target point | Space to move once set", widthScreen/2-250, 0, 22, BLACK);

		EndDrawing();
	}
	CloseWindow();

	return 0;
}

