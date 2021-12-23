#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <iostream>
#include <OpenGLHeaders/ShaderClass.h>
#include <OpenGLHeaders/texture.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#define STB_IMAGE_IMPLEMENTATION
#define ARRAY_SIZE(array) (sizeof((array)) / sizeof((array[0])))
#include <GLFW/stb_image.h>
#include <vector>
#include <odom.h>
#include <chrono>
#include <thread>
#include <cstdio>
#include "tracking.h"
#include "macros.h"
#include "autonomous.h"

// 0 if arcade, 1 if tank
#define USE_TANK 0

float deltaTime = 0.0f;
float lastFrame = 0.0f;

const float dimension = 800;

glm::mat4 zoom;
double zoomScaleFactor = 1;
const float zoomStep = 0.05f;

unsigned int driveVAO;
unsigned int driveVBO;

XDrive chassis(glm::vec2(STARTX, STARTY), STARTO);

double straight = 0;
double right = 0;;
double turn = 0;

double leftTank = 0;
double rightTank = 0;

void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
	glViewport(0, 0, width, height);
}

/**
 * Convert a coordinate on screen (in pixels) to world
*/
glm::vec2 screenToWorldCoordinates(glm::vec2 screenPos) {
	float xpos = screenPos.x;
	float ypos = screenPos.y;
	xpos -= dimension / 2;
	ypos -= dimension / 2;
	ypos = -ypos;
	screenPos = glm::vec2(xpos, ypos) * (1.0f / dimension);
	screenPos *= (float)(1.0f / zoomScaleFactor) * 2.0f;
	//screenPos = glm::vec3(glm::vec4(screenPos, 0.0f, 0.0f) * pan);
	return screenPos;
}

glm::vec2 screenToWorldCoordinates(float x, float y) {
	return screenToWorldCoordinates(glm::vec2(x, y));
}

/**
 * Process zoom commands
*/
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
	if(yoffset > 0) {
		zoom = glm::scale(zoom, glm::vec3(1.0f + zoomStep));
		zoomScaleFactor *= (1 + zoomStep);
	}
	else {
		zoom = glm::scale(zoom, glm::vec3(1.0f - zoomStep));
		zoomScaleFactor *= (1 - zoomStep);
	}
}

/**
 * Process drive commands from user
*/
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {

	double _right = 0;
	double _straight = 0;
	if (glfwGetKey(window, GLFW_KEY_W)) {
		// forward
		_straight += 1;
	}
	if (glfwGetKey(window, GLFW_KEY_S)) {
		// backward
		_straight -= 1;
	}
	if (glfwGetKey(window, GLFW_KEY_A)) {
		// left
		_right -= 1;
	}
	if (glfwGetKey(window, GLFW_KEY_D)) {
		// right
		_right += 1;
	}

	double _turn = 0;
	if(glfwGetKey(window, GLFW_KEY_RIGHT)) {
		// clockwise
		_turn -= 1;
	}
	if(glfwGetKey(window, GLFW_KEY_LEFT)) {
		// counterclockwise
		_turn += 1;
	}

	straight = _straight;
	right = _right;
	turn = _turn;
}

/**
 * Process dirve comamnds from users (controller)
*/
void fetchControllerInput() {
	int count;
	const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &count);

	// Assuming Xbox One controller with 6 axis
	float inputs[6] = {
		1, // Left joystick X axis
		-1, // Left joystick Y axis (reversed)
		1, // Right joystick X axis
		-1, // Right joystick Y axis (reversed)
		1, // LT Trigger
		1, // RT Trigger
	};

	for (int i = 0; i < count; i++) {
		inputs[i] *= axes[i];
	}

	const double DEADZONE_FIX = 0.2; // Amount to adjust for deadzone

	int leftNeg = inputs[1] < 0 ? -1 : 1;
	int rightNeg = inputs[3] < 0 ? -1 : 1;

	if (USE_TANK) {
		// Tank
		leftTank = pow(inputs[1], 3);
		rightTank = pow(inputs[3], 3);
	} else {
		// Arcade
		straight = inputs[1];
		right = 0; // None needed for skid steer drivetrain
		turn = -inputs[2] / 2 + 0.1;
	}
} 

int main()
{
	std::thread trackingThread(tracking);

	//Initialize GLFW
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	//Create a GLFW Window
	GLFWwindow *window = glfwCreateWindow(dimension, dimension, "Odometry", NULL, NULL);
	glfwMakeContextCurrent(window);

	//glad init: intializes all OpenGL function pointers
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	//Set size of rendering window
	glViewport(0, 0, dimension, dimension);

	void framebuffer_size_callback(GLFWwindow * window, int width, int height);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);	
	
	const float fieldSizeOffset = 3.5;
	float s = (144 - fieldSizeOffset)/144; // the field is 140.5 in wide but everything is on a 144 in scale

	float squareVerticesTextured[] {
		//Vertices             //Texture coords
		-s, -s, 0.0f,   0.0f, 1.0f,
		-s,  s, 0.0f,   0.0f, 0.0f,
		 s, -s, 0.0f,   1.0f, 1.0f,
		 s, -s, 0.0f,   1.0f, 1.0f,
		-s,  s, 0.0f,   0.0f, 0.0f,
		 s,  s, 0.0f,   1.0f, 0.0f
	};

	float squareVertices[] {
		//Vertices         	   //Texture coords
		-9.0f,  -9.0f, 0.0f,   0.0f, 1.0f,
		-9.0f,   9.0f, 0.0f,   0.0f, 0.0f,
		 9.0f,  -9.0f, 0.0f,   1.0f, 1.0f,
		 9.0f,  -9.0f, 0.0f,   1.0f, 1.0f,
		-9.0f,   9.0f, 0.0f,   0.0f, 0.0f,
		 9.0f,   9.0f, 0.0f,   1.0f, 0.0f
	};

	//Background data
	unsigned int backgroundVAO;
	glGenVertexArrays(1, &backgroundVAO);
	glBindVertexArray(backgroundVAO);
	unsigned int backgroundVBO;
	glGenBuffers(1, &backgroundVBO);
	glBindBuffer(GL_ARRAY_BUFFER, backgroundVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(squareVerticesTextured), squareVerticesTextured, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	//Create a Vertex Array Object
	glGenVertexArrays(1, &driveVAO);
	glBindVertexArray(driveVAO);
	
	glGenBuffers(1, &driveVBO);
	glBindBuffer(GL_ARRAY_BUFFER, driveVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(squareVertices), squareVertices, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	glm::vec3 objectColour = glm::vec3(1.0f, 0.5f, 0.31f);
	glm::vec3 lightColour = glm::vec3(1.0f, 1.0f, 1.0f);

	Shader driveShader("Shaders/VertexTexture.vs", "Shaders/FragmentTexture.fs");

	driveShader.use();
	driveShader.setInt("text", 0);
	unsigned int driveTexture;
	loadTexture(driveTexture, "textures/botTexture2.png");
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, driveTexture);

	Shader backgroundShader("Shaders/VertexTexture.vs", "Shaders/FragmentTexture.fs");

	backgroundShader.use();
	backgroundShader.setInt("text", 1);
	unsigned int backgroundTexture;
	loadTexture(backgroundTexture, "textures/VEX_TippingPoint.png");
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, backgroundTexture);

	glEnable(GL_DEPTH_TEST);
	//Set mouse input callback function
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetKeyCallback(window, key_callback);
	glPointSize(8);
	glLineWidth(3);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	std::thread autoThread(myAutonomous);

	//Render Loop
	while (!glfwWindowShouldClose(window))
	{
		// Quit
		if (glfwGetKey(window, GLFW_KEY_ESCAPE)) {
			glfwSetWindowShouldClose(window, true);
		}

		// Frame deltas
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// Clear
		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Check if auto is running before allowing manual drive
		if(!suspendDrive) {
			// Process any controller inputs
			fetchControllerInput();
			if (USE_TANK) {
				// Tank
				chassis.tank(leftTank, rightTank);
			} else {
				// ARCADE
				chassis.strafe(glm::vec2(right, straight), -turn);
			}
		}

		//Pass matrices to the shader through a uniform
		glActiveTexture(GL_TEXTURE0);
		driveShader.use();
		driveShader.setMat4("model", zoom * chassis.getMatrix());

		driveShader.setVec3("colour", glm::vec3(1.0f));
		glBindVertexArray(driveVAO);
		glBindBuffer(GL_ARRAY_BUFFER, driveVBO);
		glDrawArrays(GL_TRIANGLES, 0, ARRAY_SIZE(squareVertices));

		//Draw Background
		glm::mat4 fieldShift;
		fieldShift = glm::translate(fieldShift, glm::vec3(-fieldSizeOffset/144, -fieldSizeOffset/144, 0));
		glActiveTexture(GL_TEXTURE1);
		backgroundShader.use();
		backgroundShader.setMat4("model", zoom * fieldShift);
		glBindVertexArray(backgroundVAO);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDrawArrays(GL_TRIANGLES, 0, ARRAY_SIZE(squareVerticesTextured));

		//Swap buffer and poll IO events
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}
