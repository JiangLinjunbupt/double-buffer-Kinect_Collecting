#include "GL/freeglut.h"         //OpenGL的头文件Includ顺序会导致出现编译错误
#include "MyKinect.h"
#include "PointCloud.h"
#include <time.h>


struct Control {
	int x;
	int y;
	bool mouse_click;
	GLfloat rotx;
	GLfloat roty;
	double gx;
	double gy;
	double gz;

	Control() :x(0), y(0), rotx(0.0), roty(0.0), mouse_click(false),
		gx(0), gy(0), gz(0) {

	}
};

Control control;


Camera *camera = new Camera();
myKinect mykinect(camera);
DataFrame dataframe;
HandFinder handfinder(camera);
PointCloud pointcloud;
clock_t start;
clock_t ends_clock;

#pragma region OpenGL

#pragma region  Keybroad_event(show mesh or not)


/* executed when a regular key is pressed */
void keyboardDown(unsigned char key, int x, int y) {
	switch (key) {
	case  27:   // ESC
		exit(0);
	}
}

/* executed when a regular key is released */
void keyboardUp(unsigned char key, int x, int y) {}

#pragma endregion  Keybroad_event(show mesh or not)


/* reshaped window */
void reshape(int width, int height) {

	GLfloat fieldOfView = 90.0f;
	glViewport(0, 0, (GLsizei)width, (GLsizei)height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fieldOfView, (GLfloat)width / (GLfloat)height, 0.1, 500.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/* executed when button 'button' is put into state 'state' at screen position ('x', 'y') */
void mouseClick(int button, int state, int x, int y) {
	control.mouse_click = 1;
	control.x = x;
	control.y = y;
}

/* executed when the mouse moves to position ('x', 'y') */
/* render the scene */
void draw() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	gluPerspective(180, 1.5, -1000, 1000);
	glLoadIdentity();
	control.gx = pointcloud.PointCloud_center_x;
	control.gy = pointcloud.PointCloud_center_y;
	control.gz = pointcloud.PointCloud_center_z;
	double r = 200;
	double x = r*sin(control.roty)*cos(control.rotx);
	double y = r*sin(control.roty)*sin(control.rotx);
	double z = r*cos(control.roty);
	//cout<< x <<" "<< y <<" " << z<<endl;
	gluLookAt(x + control.gx, y + control.gy, z + control.gz, control.gx, control.gy, control.gz, 0.0, 1.0, 0.0);//个人理解最开始是看向-z的，之后的角度是在global中心上叠加的，所以要加


	//画点云
	if (pointcloud.pointcloud_vector.size() > 0)
	{
		glPointSize(2);
		glBegin(GL_POINTS);
		glColor3d(0.0, 1.0, 0.0);
		//cout << "the pointcloud size : " << pointcloud.pointcloud_vector.size() << endl;
		for (int i = 0; i < pointcloud.pointcloud_vector.size(); i++)
		{
			glVertex3f(pointcloud.pointcloud_vector[i].x(), pointcloud.pointcloud_vector[i].y(), pointcloud.pointcloud_vector[i].z());
		}
		glEnd();
	}

	glFlush();
	glutSwapBuffers();
}


void mouseMotion(int x, int y) {
	control.rotx = (x - control.x)*0.05;
	control.roty = (y - control.y)*0.05;

	//cout<< control.rotx <<" " << control.roty << endl;
	glutPostRedisplay();
}

void idle() {
	start = clock();


	mykinect.fetch_data(dataframe, handfinder,pointcloud);

	cv::imshow("depth_show", handfinder.sensor_hand_silhouette);
	cv::waitKey(1);

	ends_clock = clock();
	cout << "Running Time : " << (double)(ends_clock - start) / CLOCKS_PER_SEC << endl;

	//Sleep(30);
	glutPostRedisplay();
}
/* initialize OpenGL settings */
void initGL(int width, int height) {

	reshape(width, height);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
}

#pragma endregion 


int main(int argc, char** argv) 
{
	HRESULT hr = mykinect.InitializeDefaultSensor();
	pointcloud.camera = camera;

	if (FAILED(hr))
	{
		return -1;
	}

	//for (;;)
	//{
	//	mykinect.getData();
	//}
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Interactron");

	// register glut call backs
	glutKeyboardFunc(keyboardDown);
	glutKeyboardUpFunc(keyboardUp);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMotion);
	glutReshapeFunc(reshape);
	glutDisplayFunc(draw);
	glutIdleFunc(idle);
	glutIgnoreKeyRepeat(true); // ignore keys held down


	initGL(800, 600);

	glutMainLoop();

	return 0;

}