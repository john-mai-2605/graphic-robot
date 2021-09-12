////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

#include <GL/glew.h>
#ifdef __APPLE__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "cvec.h"
#include "matrix4.h"
#include "rigtform.h"
#include "quat.h"
#include "arcball.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"
#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"
#include <list>
#include "sgutils.h"
#include <fstream>
#include "geometry.h"
#include "mesh.h"

using namespace std;      // for string, vector, iostream, and other standard C++ stuff

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
const bool g_Gl2Compatible = false;


static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;

static int eyeIndex = 0;
static int sky_skyIndex = 1;
static int pickMode = 0;

typedef vector<RigTForm> KeyFrame;
list<KeyFrame> keyFrames;
int g_currentKeyFrameIndex = 0;
list<KeyFrame>::iterator g_currentKeyFrame = keyFrames.begin();
list<KeyFrame>::iterator g_PreKeyFrame;
list<KeyFrame>::iterator g_FromKeyFrame;
list<KeyFrame>::iterator g_ToKeyFrame;
list<KeyFrame>::iterator g_PostKeyFrame;
static int g_msBetweenKeyFrames = 2000; // 2 seconds between keyframes
static int g_animateFramesPerSecond = 60; // frames to render per second during animation playback
static int g_num_interp = g_animateFramesPerSecond * g_msBetweenKeyFrames / 1000;
static int g_isPlaying = 0;
static int g_mostRecentKeyFrameIndex = 0;
static Mesh g_referenceMesh = Mesh();
static Mesh g_tempMesh;
static shared_ptr<SimpleGeometryPN> g_subdividedMesh;
static int smoothMode = 0;
static float g_msBetweenMeshChange = 100;
static float g_meshFreq = 1000;
static int g_step = 0;
// --------- Materials
// This should replace all the contents in the Shaders section, e.g., g_numShaders, g_shaderFiles, and so on
static shared_ptr<Material> g_redDiffuseMat,
g_blueDiffuseMat,
g_bumpFloorMat,
g_arcballMat,
g_pickingMat,
g_lightMat,
g_specularMat;

shared_ptr<Material> g_overridingMaterial;
// ===================================================================
// Declare the scene graph and pointers to suitable nodes in the scene
// graph
// ===================================================================

static const int PICKING_SHADER = 2; // index of the picking shader is g_shaerFiles


// --------- Geometry
typedef SgGeometryShapeNode MyShapeNode;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere;

// --------- Scene

static const Cvec3 g_light1(4.0, 2.5, 1.0), g_light2(-4, 2.5, -1.0);  // define two lights positions in world space
static char* objName[3] = {"Sky", "Robot 1", "Robot 2"};

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node, g_light1Node, g_light2Node, g_subdividedMeshNode;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode; // used later when you do picking
static float g_arcballScreenRadius = 0.25 * min(g_windowWidth, g_windowHeight);
static float g_arcballScale;

///////////////// END OF G L O B A L S //////////////////////////////////////////////////

// ---------- Asst 8 (Mesh)
static vector<VertexPN> getNormalAllVertices(Mesh &mesh) {
    // set normal
    for (int i = 0; i < mesh.getNumVertices(); i++) {
        Mesh::VertexIterator currentVertex = mesh.getVertex(i).getIterator();
        const Mesh::VertexIterator baseVertex = currentVertex;
        ++currentVertex;
        Cvec3 accum = baseVertex.getFace().getNormal();
        while (currentVertex != baseVertex) {
            accum += currentVertex.getFace().getNormal();
            ++currentVertex;
        }
        if (norm(accum) > CS175_EPS) accum.normalize();
        mesh.getVertex(i).setNormal(accum);
    }

    // return all normals in a vector
    vector<VertexPN> result;
    if (!smoothMode) {
        // flat shading
        for (int i = 0; i < mesh.getNumFaces(); i++) {
            Mesh::Face face = mesh.getFace(i);
            for (int j = 1; j < face.getNumVertices() - 1; j++) {
                result.push_back(VertexPN(face.getVertex(0).getPosition(), face.getNormal()));
                result.push_back(VertexPN(face.getVertex(j).getPosition(), face.getNormal()));
                result.push_back(VertexPN(face.getVertex(j + 1).getPosition(), face.getNormal()));
            }
        }
        return result;
    }
    // smooth shading
    for (int i = 0; i < mesh.getNumFaces(); i++) {
        Mesh::Face face = mesh.getFace(i);
        for (int j = 1; j < face.getNumVertices() - 1; j++) {
            result.push_back(VertexPN(face.getVertex(0).getPosition(), face.getVertex(0).getNormal()));
            result.push_back(VertexPN(face.getVertex(j).getPosition(), face.getVertex(j).getNormal()));
            result.push_back(VertexPN(face.getVertex(j + 1).getPosition(), face.getVertex(j+1).getNormal()));
        }
    }
    return result;
}

static void subdivision(Mesh &mesh, int step) {
    for (int st = 0; st < step; st++) {
        for (int i = 0; i < mesh.getNumFaces(); i++) {
            Mesh::Face face = mesh.getFace(i);
            Cvec3 accum = Cvec3(0, 0, 0);
            for (int j = 0; j < face.getNumVertices(); j++) accum += face.getVertex(j).getPosition();
            Cvec3 geometry = accum / face.getNumVertices();
            mesh.setNewFaceVertex(face, geometry);
        }

        for (int i = 0; i < mesh.getNumEdges(); i++) {
            Mesh::Edge edge = mesh.getEdge(i);
            Cvec3 accum = edge.getVertex(0).getPosition() + edge.getVertex(1).getPosition();
            accum += mesh.getNewFaceVertex(edge.getFace(0)) + mesh.getNewFaceVertex(edge.getFace(1));
            Cvec3 geometry = accum / 4.0;
            mesh.setNewEdgeVertex(edge, geometry);
        }

        for (int i = 0; i < mesh.getNumVertices(); i++) {
            Mesh::VertexIterator currentVertex = mesh.getVertex(i).getIterator();
            Mesh::VertexIterator baseVertex = currentVertex;
            ++currentVertex;
            int numVertex = 1;
            Cvec3 accum = baseVertex.getVertex().getPosition();
            Cvec3 accumFace = mesh.getNewFaceVertex(baseVertex.getFace());
            while (currentVertex != baseVertex) {
                accum += currentVertex.getVertex().getPosition();
                accumFace += mesh.getNewFaceVertex(currentVertex.getFace());
                ++currentVertex;
                numVertex++;
            }
            Cvec3 geometry = mesh.getVertex(i).getPosition() * (1.0 - 2.0 / numVertex) + (accum + accumFace) / ((float)numVertex * numVertex);
            mesh.setNewVertexVertex(mesh.getVertex(i), geometry);
        }
        mesh.subdivide();
    }
}

static void uploadMesh() {
    //set up reference
    g_referenceMesh.load("cube.mesh");
    vector<VertexPN> verticeNormals = getNormalAllVertices(g_referenceMesh);
    // copy to temporary mesh
    g_tempMesh = Mesh(g_referenceMesh);
    // set the mesh geometry to the saved vertices
    g_subdividedMesh = make_unique<SimpleGeometryPN>();
    g_subdividedMesh->upload(&verticeNormals.at(0), verticeNormals.size());
}

static void animateMeshCallback(int ms) {
    float t = (float)ms / g_msBetweenMeshChange;
    g_tempMesh = Mesh(g_referenceMesh);
    for (int i = 0; i < g_referenceMesh.getNumVertices(); i++) {
        // scale the object coordinates of each of the cube’s vertices by a periodic, time-varying scalar
        const Cvec3 referenceVertex = g_referenceMesh.getVertex(i).getPosition();
        const Mesh::Vertex tempVertex = g_tempMesh.getVertex(i);
        tempVertex.setPosition(referenceVertex * (1 + 0.5 * sin((t + 120 * i) / 150)));
    }
    subdivision(g_tempMesh, g_step);
    vector<VertexPN> verticeNormals = getNormalAllVertices(g_tempMesh);
    g_subdividedMesh->upload(&verticeNormals.at(0), verticeNormals.size());

    glutPostRedisplay();
    glutTimerFunc(10,
        animateMeshCallback,
        ms + g_meshFreq);
}

//=======================================================================
static void initGround() {
    int ibLen, vbLen;
    getPlaneVbIbLen(vbLen, ibLen);

    // Temporary storage for cube Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);

    makePlane(g_groundSize * 2, vtx.begin(), idx.begin());
    g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes() {
    int ibLen, vbLen;
    getCubeVbIbLen(vbLen, ibLen);

    // Temporary storage for cube Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);

    makeCube(1, vtx.begin(), idx.begin());
    g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
    int ibLen, vbLen;
    getSphereVbIbLen(20, 10, vbLen, ibLen);

    // Temporary storage for sphere Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);
    makeSphere(1, 20, 10, vtx.begin(), idx.begin());
    g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

// takes a projection matrix and send to the the shaders
inline void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
    uniforms.put("uProjMatrix", projMatrix);
}


// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
  if (g_windowWidth >= g_windowHeight)
    g_frustFovY = g_frustMinFov;
  else {
    const double RAD_PER_DEG = 0.5 * CS175_PI/180;
    g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
  }
}

static Matrix4 makeProjectionMatrix() {
  return Matrix4::makeProjection(
           g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
           g_frustNear, g_frustFar);
}

bool hasArcBallInSkySkyView() {
    return (eyeIndex == 0 && !g_currentPickedRbtNode && sky_skyIndex == 1);
}

bool hasArcBall() {
    static shared_ptr<SgRbtNode> viewNodes[3] = { g_skyNode, g_robot1Node, g_robot2Node };
    return hasArcBallInSkySkyView() || (g_currentPickedRbtNode && g_currentPickedRbtNode != viewNodes[eyeIndex]);
}
static void drawStuff(bool picking) {
    // Declare an empty uniforms
    Uniforms uniforms;

    // build & send proj. matrix to vshader
    const Matrix4 projmat = makeProjectionMatrix();
    sendProjectionMatrix(uniforms, projmat);
 

    static shared_ptr<SgRbtNode> viewNodes[3] = { g_skyNode, g_robot1Node, g_robot2Node };
    RigTForm eyeRbt = getPathAccumRbt(g_world, viewNodes[eyeIndex]);
    RigTForm invEyeRbt = inv(eyeRbt);
    RigTForm objRbt = g_currentPickedRbtNode ? getPathAccumRbt(g_world, g_currentPickedRbtNode) : RigTForm();

    Cvec3 light1 = getPathAccumRbt(g_world, g_light1Node).getTranslation();
    Cvec3 light2 = getPathAccumRbt(g_world, g_light2Node).getTranslation();
    const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(light1, 1)); // g_light1 position in eye coordinates
    const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(light2, 1)); // g_light2 position in eye coordinates
    uniforms.put("uLight", eyeLight1);
    uniforms.put("uLight2", eyeLight2);

    if ( !g_mouseMClickButton && !(g_mouseLClickButton && g_mouseRClickButton)) {
        g_arcballScale = hasArcBall() ? getScreenToEyeScale((invEyeRbt * objRbt).getTranslation()[2], g_frustFovY, g_windowHeight) : 0.01;
    }

    Matrix4 MVM, NMVM;

    if (!picking) {
        Drawer drawer(invEyeRbt, uniforms);
        g_world->accept(drawer);

        // draw arcball as part of asst3
        if (hasArcBall()) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            double radius = g_arcballScreenRadius * g_arcballScale;
            MVM = rigTFormToMatrix(invEyeRbt * objRbt) * Matrix4::makeScale(Cvec3(radius, radius, radius));
            NMVM = normalMatrix(MVM);
            sendModelViewNormalMatrix(uniforms, MVM, normalMatrix(MVM));
            g_arcballMat->draw(*g_sphere, uniforms);
        }
    }
    else if (g_mouseLClickButton && !g_mouseRClickButton) {
        // intialize the picker with our uniforms, as opposed to curSS
        Picker picker(invEyeRbt, uniforms);
        // set overiding material to our picking material
        g_overridingMaterial = g_pickingMat;

        // draw
        g_world->accept(picker);

        // unset the overriding material
        g_overridingMaterial.reset();

        glFlush();
        g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
        if (g_currentPickedRbtNode == g_groundNode ) {
            g_currentPickedRbtNode = shared_ptr<SgRbtNode>();   // set to NULL
        }
   
        if (g_currentPickedRbtNode) cout << "Part picked\n";
        else cout << "No part picked\n";
        pickMode = 0;
        cout << "Turn off picking mode \n";
    }
}

static void pick() {
    // We need to set the clear color to black, for pick rendering.
    // so let's save the clear color
    GLdouble clearColor[4];
    glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

    glClearColor(0, 0, 0, 0);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // No more glUseProgram
    drawStuff(true); // no more curSS

    // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
    // to see result of the pick rendering pass
    // glutSwapBuffers();

    //Now set back the clear color
    glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

    checkGlErrors();

}

static void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

  if (!pickMode) {
      drawStuff(pickMode);

      glutSwapBuffers();                                    // show the back buffer (where we rendered stuff)
  }
  else {
      pick();
  }

  checkGlErrors();
}

static void reshape(const int w, const int h) {
  g_windowWidth = w;
  g_windowHeight = h;
  g_arcballScreenRadius = 0.25 * min(g_windowWidth, g_windowHeight);
  glViewport(0, 0, w, h);
  cerr << "Size of window is now " << w << "x" << h << endl;
  updateFrustFovY();
  glutPostRedisplay();
}

static void motion(const int x, const int y) {
  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;

  static shared_ptr<SgRbtNode> viewNodes[3] = { g_skyNode, g_robot1Node, g_robot2Node };
  RigTForm eyeRbt =  getPathAccumRbt(g_world, viewNodes[eyeIndex]); //objectsList[eyeIndex];
  RigTForm invEyeRbt = inv(eyeRbt);
  shared_ptr<SgRbtNode> pickedObj = g_currentPickedRbtNode ? g_currentPickedRbtNode : viewNodes[eyeIndex];
  RigTForm objRbt = g_currentPickedRbtNode ? getPathAccumRbt(g_world, g_currentPickedRbtNode) : eyeRbt;

  RigTForm m;
  if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
      if (!hasArcBall()) m = RigTForm(Quat::makeXRotation(-dy) * Quat::makeYRotation(dx));
      else {
          Cvec2 s1 = Cvec2(g_mouseClickX, g_mouseClickY);
          Cvec2 s2 = Cvec2(x, g_windowHeight - y - 1);
          Cvec3 O      ;
          if (g_currentPickedRbtNode)
              O = RigTForm(invEyeRbt * objRbt).getTranslation();
          else
              O = RigTForm(invEyeRbt).getTranslation();

          Cvec2 O1 = getScreenSpaceCoord(O, makeProjectionMatrix(), g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);
          double r = g_arcballScreenRadius;
          s1 = s1 - O1;
          s2 = s2 - O1;
          double scale = norm(s1) / r;
          if (scale > 1) {
              s1 /= scale;
          }
          scale = norm(s2) / r;
          if (scale > 1) {
              s2 /= scale;
          }
          double const z1 = max(r * r - norm2(s1), CS175_EPS);
          double const z2 = max(r * r - norm2(s2), CS175_EPS);
          Cvec3 p1 = Cvec3(s1, sqrt(z1));
          Cvec3 p2 = Cvec3(s2, sqrt(z2));
          p1 /= norm(p1);
          p2 /= norm(p2);
          m = RigTForm(Quat(dot(p1, p2), cross(p1, p2)));
      }
  }
  else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
      m = RigTForm(Cvec3(dx, dy, 0) * g_arcballScale);
  }
  else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
      m = RigTForm(Cvec3(0, 0, -dy) * g_arcballScale);
  }

  if (hasArcBallInSkySkyView()) m = inv(m);
  else if (!g_currentPickedRbtNode || g_currentPickedRbtNode == viewNodes[eyeIndex]) m.setRotation(inv(m.getRotation()));

  if (g_mouseClickDown) {

      RigTForm frame = RigTForm(transFact(objRbt).getTranslation(), linFact(eyeRbt).getRotation());
      if (hasArcBallInSkySkyView()) frame = RigTForm(linFact(eyeRbt));

      RigTForm C = g_currentPickedRbtNode ? getPathAccumRbt(g_world, g_currentPickedRbtNode, 1) : RigTForm();
      RigTForm As = inv(C) * frame;
      RigTForm L = pickedObj -> getRbt();

      pickedObj -> setRbt(As * m * inv(As) * L);
      glutPostRedisplay(); // we always redraw if we changed the scene
      
  }

  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;
}


static void mouse(const int button, const int state, const int x, const int y) {
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

  g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
  g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
  g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

  g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
  g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
  g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

  g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;

  glutPostRedisplay();
}

static void copyCurrentKeyFrame() {
    if (!keyFrames.empty()) {
        vector<shared_ptr<SgRbtNode> > rbtNodes;
        dumpSgRbtNodes(g_world, rbtNodes);
        for (int i = 0; i < rbtNodes.size(); i++) {
            rbtNodes[i]->setRbt((*g_currentKeyFrame)[i]);
        }
    }
}
static void makeNewKeyFrame() {
    KeyFrame newFrame;
    if (keyFrames.empty()) {
        keyFrames.push_back(newFrame);
        g_currentKeyFrame = keyFrames.begin();
        g_currentKeyFrameIndex = 0;
    }
    else if (g_currentKeyFrame == keyFrames.end()) {
        keyFrames.push_back(newFrame);
        g_currentKeyFrame--;
        g_currentKeyFrameIndex--;
    }
    else {
        // since the frame is inserted AFTER, we need to increment the current pointer
        keyFrames.insert(++g_currentKeyFrame, newFrame);
        // and move back
        g_currentKeyFrame--;
        g_currentKeyFrameIndex++;
    }
    vector<shared_ptr<SgRbtNode> > rbtNodes;
    dumpSgRbtNodes(g_world, rbtNodes);
    g_currentKeyFrame->clear();
    for (int i = 0; i < rbtNodes.size(); i++) {
        g_currentKeyFrame->push_back(rbtNodes[i]->getRbt());
    }
    printf("Successfully! Currently at KeyFrame [%d]\n", g_currentKeyFrameIndex);
}
static void updateKeyFrame() {
    if (keyFrames.empty()) {
        makeNewKeyFrame();
    }
    else {
        vector<shared_ptr<SgRbtNode> > rbtNodes;
        dumpSgRbtNodes(g_world, rbtNodes);
        g_currentKeyFrame->clear();
        for (int i = 0; i < rbtNodes.size(); i++) {
            g_currentKeyFrame->push_back(rbtNodes[i]->getRbt());
        }
    }
}
static void advanceKeyFrame() {
    if (g_currentKeyFrame == keyFrames.end()) return;
    g_currentKeyFrame++;
    g_currentKeyFrameIndex++;
    if (g_currentKeyFrame == keyFrames.end()) {
        printf("Reach the end!\n");
        g_currentKeyFrame--;
        g_currentKeyFrameIndex--;
        return;
    }
    copyCurrentKeyFrame();
    printf("Successfully! Currently at KeyFrame [%d]\n", g_currentKeyFrameIndex);
}
static void retreatKeyFrame() {
    if (g_currentKeyFrame == keyFrames.begin()) {
        printf("Reach the start!\n");
        return;
    }
    g_currentKeyFrame--;
    g_currentKeyFrameIndex--;
    copyCurrentKeyFrame();
    printf("Successfully! Currently at KeyFrame [%d]\n", g_currentKeyFrameIndex);
}
static void deleteKeyFrame() {
    if (keyFrames.empty()) {
        printf("No KeyFrame to delete!\n");
            return;
    }
    list<KeyFrame>::iterator keyFrameAfterDel = g_currentKeyFrame;
    int temp = g_currentKeyFrameIndex;
    if (g_currentKeyFrame != keyFrames.begin()) keyFrameAfterDel--;
    else keyFrameAfterDel++;
    if (g_currentKeyFrame != keyFrames.begin()) temp--;
    else temp++;
    keyFrames.erase(g_currentKeyFrame);

    if (keyFrames.empty()) g_currentKeyFrame = keyFrames.end();
    else g_currentKeyFrame = keyFrameAfterDel;
    if (keyFrames.empty()) g_currentKeyFrameIndex = -1;
    else g_currentKeyFrameIndex = temp;
    copyCurrentKeyFrame();
    printf("Successfully! Currently at KeyFrame [%d]\n", g_currentKeyFrameIndex);
}
static void inputKeyFrame(){
    const char* filename = "keyframes.txt";
    int numFrames, numRbtsPerFrame;
    ifstream f(filename, ios::binary);
    f >> numFrames >> numRbtsPerFrame;
    //cout << numFrames << ' ' << numRbtsPerFrame << '\n';

    keyFrames.clear();
    for (int i = 0; i < numFrames; i++) {
        KeyFrame frame;
        for (int j = 0; j < numRbtsPerFrame; j++) {
            Cvec3 T;
            Quat R;
            f >> T[0] >> T[1] >> T[2];
            f >> R[0] >> R[1] >> R[2] >> R[3];
            // cout << T[0] << ' ' << T[1] << ' ' << T[2] << '\n';
            // cout << R[0] << ' ' << R[1] << ' ' << R[2] << ' ' << R[3] << '\n';
            frame.push_back(RigTForm(T, R));
        }
        keyFrames.push_back(frame);
    }
    cout << "Done inputing " << numFrames << " KeyFrame from file " << filename << '\n';
    g_currentKeyFrame = keyFrames.begin();
}

static void outputKeyFrame() {
    if (keyFrames.empty()) {
        printf("No KeyFrame to output!\n");
        return;
    }
    const char* filename = "keyframes.txt";
    int numFrames, numRbtsPerFrame;
    ofstream f(filename, ios::binary);

    numFrames = keyFrames.size();
    list<KeyFrame>::iterator frameIter = keyFrames.begin();
    numRbtsPerFrame = (*frameIter).size();
    f << numFrames << ' ' << numRbtsPerFrame << '\n';
    for (frameIter; frameIter != keyFrames.end(); frameIter++) {
        KeyFrame keyFrame = *frameIter;
        for (int i = 0; i < keyFrame.size(); i++) {
            RigTForm rbt = keyFrame[i];
            Cvec3 T = rbt.getTranslation();
            Quat R = rbt.getRotation();
            f << T[0] << ' ' << T[1] << ' ' << T[2] << '\n';
            f << R[0] << ' ' << R[1] << ' ' << R[2] << ' ' << R[3] << '\n';
        }
    }
    cout << "Done outputing " << numFrames << " KeyFrame to file " << filename << '\n';
}
// Given t in the range [0, n], perform interpolation and draw the scene
// for the particular t. Returns true if we are at the end of the animation
// sequence, or false otherwise.
bool interpolateAndDisplay(float t) { 
    int keyFrameIndex = floor(t);
    list<KeyFrame>::iterator finalKeyFrame = keyFrames.end();
    finalKeyFrame--;
    double alpha = t - keyFrameIndex;
    if (!g_isPlaying) return true;
    if (keyFrameIndex == 0) {
        g_PreKeyFrame = keyFrames.begin();
        g_FromKeyFrame = g_PreKeyFrame;
        g_FromKeyFrame++;
        g_ToKeyFrame = g_FromKeyFrame;
        g_ToKeyFrame++;
        g_PostKeyFrame = g_ToKeyFrame;
        g_PostKeyFrame++;
    }
    if (keyFrameIndex != g_mostRecentKeyFrameIndex) {
        g_PreKeyFrame++;
        g_FromKeyFrame++;
        g_ToKeyFrame++;
        g_PostKeyFrame++;
        g_mostRecentKeyFrameIndex = keyFrameIndex;
    }
    if (g_ToKeyFrame == finalKeyFrame) {
        g_isPlaying = 0;
        g_mostRecentKeyFrameIndex = 0;
        cout << "End animation\n";
        g_currentKeyFrameIndex = keyFrameIndex+1;
        g_currentKeyFrame = g_FromKeyFrame;
        printf("Currently at KeyFrame [%d]\n", g_currentKeyFrameIndex);
        return true;
    }
    KeyFrame interpFrame;
    for (int i = 0; i < g_FromKeyFrame->size(); i++) {
        RigTForm preRbt = (*g_PreKeyFrame)[i];
        RigTForm fromRbt = (*g_FromKeyFrame)[i];
        RigTForm toRbt = (*g_ToKeyFrame)[i];
        RigTForm postRbt = (*g_PostKeyFrame)[i];
        Cvec3 preT = preRbt.getTranslation();
        Cvec3 fromT = fromRbt.getTranslation();
        Cvec3 toT = toRbt.getTranslation();
        Cvec3 postT = postRbt.getTranslation();
        Quat preR = preRbt.getRotation();
        Quat fromR = fromRbt.getRotation();
        Quat toR = toRbt.getRotation();
        Quat postR = postRbt.getRotation();
        RigTForm interpRbt = RigTForm(crlerp(preT, fromT, toT, postT, alpha), crslerp(preR, fromR, toR, postR, alpha));
        interpFrame.push_back(interpRbt);
    }
    vector<shared_ptr<SgRbtNode> > rbtNodes;
    dumpSgRbtNodes(g_world, rbtNodes);
    for (int i = 0; i < rbtNodes.size(); i++) {
        rbtNodes[i]->setRbt(interpFrame[i]);
    }
    glutPostRedisplay();
    return false;
}
// Interpret "ms" as milliseconds into the animation
static void animateTimerCallback(int ms) {
    float t = (float)ms / (float)g_msBetweenKeyFrames;
    bool endReached = interpolateAndDisplay(t);
    if (!endReached)
        glutTimerFunc(1000 / g_animateFramesPerSecond,
            animateTimerCallback,
            ms + 1000 / g_animateFramesPerSecond);
    else { 
        g_isPlaying = 0;
    }
}

static void keyboard(const unsigned char key, const int x, const int y) {
    switch (key) {
    case 27:
        exit(0);                                  // ESC
    case 'h':
        cout << " ============== H E L P ==============\n\n"
            << "h\t\thelp menu\n"
            << "s\t\tsave screenshot\n"
            << "f\t\tToggle flat shading on/off.\n"
            << "p\t\tTurn on picker mode\n"
            << "v\t\tCycle view\n"
            << "drag left mouse to rotate\n" 
            << "drag right mouse to rotate x-y axis\n"
            << "drag middle mouse to rotate z axis\n"
            << endl;
        break;
    case 's':
        glFlush();
        writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
        break;


    case 'v':
        eyeIndex = (eyeIndex + 1) % 3;
        cout << objName[eyeIndex] << " is active eye\n";
        break;

    case 'm':
        sky_skyIndex = 1 - sky_skyIndex;
        if (sky_skyIndex == 1) {
            cout << "Editing sky eye w.r.t. World - sky frame \n";
        }
        else {
            cout << "Editing sky eye w.r.t. Sky - sky frame \n";
        }
        break;
    case 'p':
        pickMode = 1 - pickMode;
        if (pickMode) cout << "Turn on picking mode \n";
        else cout << "Turn off picking mode \n";
        break;
    case ' ':
        cout << "Copy the current Keyframe into the scene\n";
        copyCurrentKeyFrame();
        break;
    case 'u':
        cout << "Update the current Keyframe with the scene\n";
        updateKeyFrame();
        break;
    case '>':
        cout << "Try to advance to the next KeyFrame...\n";
        advanceKeyFrame();
        break;
    case '<':
        cout << "Try to retreat to the previous KeyFrame...\n";
        retreatKeyFrame();
        break;
    case 'd':
        cout << "Delete the current KeyFrame...\n";
        deleteKeyFrame();
        break;
    case 'n':
        cout << "Create a new KeyFrame...\n";
        makeNewKeyFrame();
        break;
    case 'i':
        cout << "Read from file ...\n";
        inputKeyFrame();
        break;
    case 'w':
        cout << "Write to file ...\n";
        outputKeyFrame();
        break;
    case 'y':
        g_isPlaying = 1 - g_isPlaying;
        if (g_isPlaying) {
            if (keyFrames.size() < 4) {
                cout << "Not enough keyframes! Need 4, have " << keyFrames.size() << '\n';
                g_isPlaying = 1 - g_isPlaying;
            }
            else {
                g_currentKeyFrame = keyFrames.begin();
                g_currentKeyFrame++;
                g_currentKeyFrameIndex = 1;
                cout << "Start playing animation...\n";
                animateTimerCallback(0);
            }
        }
        else {
            g_currentKeyFrame = keyFrames.end();
            g_currentKeyFrameIndex = keyFrames.size() - 2;
            g_currentKeyFrame--; 
            g_currentKeyFrame--;
            cout << "Stop playing animation\n";
            printf("Currently at keyFrame [%d]\n", g_currentKeyFrameIndex);
            copyCurrentKeyFrame();
        }
        break;
    case '+':
        if (g_num_interp <= 1) cout << "Cannot speed up anymore\n";
        else {
            g_num_interp--;
            g_msBetweenKeyFrames = g_num_interp * 1000 / g_animateFramesPerSecond;
            cout << "Number of interpolated frames is " << g_num_interp << " between KeyFrames\n";
        }
        break;
    case '-':
        g_num_interp++;
        g_msBetweenKeyFrames = g_num_interp * 1000 / g_animateFramesPerSecond;
        cout << "Number of interpolated frames is " << g_num_interp << " between KeyFrames\n";
     
        break;
    case 'f':
        smoothMode = 1-smoothMode;
        if (smoothMode) cout << "Smooth shading\n";
        break;
    case '0':
        g_step = min(g_step + 1, 7);
        cout << g_step << " subdivisions\n";
        break;
    case '9':
        g_step = max(g_step - 1, 0);
        cout << g_step << " subdivisions\n";
    break; 
    case '7':
        g_meshFreq = g_meshFreq/2;
        cout << "Half the speed\n";
        break;
    case '8':
        g_meshFreq = g_meshFreq*2;
        cout << "Double the speed\n";
        break;
    }



  glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 5 - John");                       // title the window

  glutDisplayFunc(display);                               // display rendering callback
  glutReshapeFunc(reshape);                               // window reshape callback
  glutMotionFunc(motion);                                 // mouse movement callback
  glutMouseFunc(mouse);                                   // mouse click callback
  glutKeyboardFunc(keyboard);
}

static void initGLState() {
  glClearColor(128./255., 200./255., 255./255., 0.);
  glClearDepth(0.);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glReadBuffer(GL_BACK);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initMaterials() {
    // Create some prototype materials
    Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
    Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");
    Material specular("./shaders/basic-gl3.vshader", "./shaders/specular-gl3.fshader");

    // copy diffuse prototype and set red color
    g_redDiffuseMat.reset(new Material(diffuse));
    g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

    // copy diffuse prototype and set blue color
    g_blueDiffuseMat.reset(new Material(diffuse));
    g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

    // normal mapping material
    g_bumpFloorMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
    g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("Fieldstone.ppm", true)));
    g_bumpFloorMat->getUniforms().put("uTexNormal", shared_ptr<ImageTexture>(new ImageTexture("FieldstoneNormal.ppm", false)));

    // copy solid prototype, and set to wireframed rendering
    g_arcballMat.reset(new Material(solid));
    g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
    g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // copy solid prototype, and set to color white
    g_lightMat.reset(new Material(solid));
    g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));

    // copy solid prototype, and set to color white
    g_specularMat.reset(new Material(specular));
    g_specularMat->getUniforms().put("uColor", Cvec3f(0.5, 0.5, 1));

    // pick shader
    g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));
};

static void initGeometry() {
  initGround();
  initCubes();
  initSphere();
  uploadMesh();
  animateMeshCallback(0);
}
// =================================================================
// Insert the following constructRobot and initScene() after the
// initGeometry() function, and call initScene() after initGeometry()
// in your main()
// =================================================================

static void constructRobot(shared_ptr<SgTransformNode> base, shared_ptr<Material> material) {

    const double ARM_LEN = 0.7,
        ARM_THICK = 0.25,
        LEG_LEN = 1,
        LEG_THICK = 0.25,
        TORSO_LEN = 1.5,
        HEAD_RADIUS = 0.25,
        NECK_LENGTH = 0.1,
        TORSO_THICK = 0.25,
        TORSO_WIDTH = 1;
    const int NUM_JOINTS = 10,
        NUM_SHAPES = 10;

    struct JointDesc {
        int parent;
        float x, y, z;
    };

    JointDesc jointDesc[NUM_JOINTS] = {
      {-1}, // torso
      {0,  TORSO_WIDTH / 2, TORSO_LEN / 2, 0}, // upper right arm
      {1,  ARM_LEN, 0, 0}, // lower right arm
      {0,  -TORSO_WIDTH / 2, TORSO_LEN / 2, 0}, // upper left arm
      {3,  -ARM_LEN, 0, 0}, // lower left arm
      {0,  TORSO_WIDTH/2 - LEG_THICK/2, -TORSO_LEN / 2, 0}, // upper right leg
      {5,  0, -LEG_LEN, 0}, // lower right leg
      {0,  -TORSO_WIDTH/2 + LEG_THICK/2, -TORSO_LEN / 2, 0}, // upper left leg
      {7,  0, -LEG_LEN, 0}, // lower left leg
      {0,  0, TORSO_LEN / 2 + HEAD_RADIUS + NECK_LENGTH, 0}, // head
    };

    struct ShapeDesc {
        int parentJointId;
        float x, y, z, sx, sy, sz;
        shared_ptr<Geometry> geometry;
    };

    ShapeDesc shapeDesc[NUM_SHAPES] = {
      {0, 0,         0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube}, // torso
      {1, ARM_LEN / 2, 0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK, g_sphere}, // upper right arm
      {2, ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower right arm
      {3, -ARM_LEN / 2, 0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK, g_sphere}, // upper left arm
      {4, -ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower left arm
      {5, LEG_LEN / 2 - TORSO_WIDTH/2, -LEG_LEN / 2, 0, LEG_THICK/2, LEG_LEN / 2, LEG_THICK, g_sphere}, // upper right leg
      {6, LEG_LEN / 2 - TORSO_WIDTH/2, -LEG_LEN / 2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower right leg
      {7, -LEG_LEN / 2 + TORSO_WIDTH/2, -LEG_LEN / 2, 0, LEG_THICK/2, LEG_LEN / 2, LEG_THICK, g_sphere}, // upper left leg
      {8, -LEG_LEN / 2 + TORSO_WIDTH / 2, - LEG_LEN / 2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower left leg
      {9, 0, 0, 0, HEAD_RADIUS,  HEAD_RADIUS, HEAD_RADIUS, g_sphere}, // head
    };

    shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (jointDesc[i].parent == -1)
            jointNodes[i] = base;
        else {
            jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
            jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
        }
    }
    for (int i = 0; i < NUM_SHAPES; ++i) {
        shared_ptr<SgGeometryShapeNode> shape(
            new MyShapeNode(shapeDesc[i].geometry,
                material, // USE MATERIAL as opposed to color
                Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
                Cvec3(0, 0, 0),
                Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
        jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
    }
}

static void initScene() {
    g_world.reset(new SgRootNode());

    g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));

    g_groundNode.reset(new SgRbtNode());
    g_groundNode->addChild(shared_ptr<MyShapeNode>(
        new MyShapeNode(g_ground, g_bumpFloorMat, Cvec3(0, g_groundY, 0))));


    g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
    g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

    constructRobot(g_robot1Node, g_redDiffuseMat); // a Red robot
    constructRobot(g_robot2Node, g_blueDiffuseMat); // a Blue robot

    g_light1Node.reset(new SgRbtNode(RigTForm(g_light1)));
    g_light2Node.reset(new SgRbtNode(RigTForm(g_light2)));
    g_light1Node->addChild(shared_ptr<MyShapeNode>(
        new MyShapeNode(g_sphere, g_lightMat, Cvec3(0, 0, 0), Cvec3(0, 0, 0),  Cvec3(0.25, 0.25, 0.25))));
    g_light2Node->addChild(shared_ptr<MyShapeNode>(
        new MyShapeNode(g_sphere, g_lightMat, Cvec3(0, 0, 0), Cvec3(0, 0, 0), Cvec3(0.25, 0.25, 0.25))));
    g_subdividedMeshNode.reset(new SgRbtNode(RigTForm()));
    g_subdividedMeshNode->addChild(shared_ptr<MyShapeNode>(
        new MyShapeNode(g_subdividedMesh, g_specularMat, Cvec3(0, 0, 0))));
    g_world->addChild(g_skyNode);
    g_world->addChild(g_groundNode);
    g_world->addChild(g_robot1Node);
    g_world->addChild(g_robot2Node);
    g_world->addChild(g_light1Node);
    g_world->addChild(g_light2Node);
    g_world->addChild(g_subdividedMeshNode);
}

int main(int argc, char * argv[]) {
  try {
    initGlutState(argc,argv);

    glewInit(); // load the OpenGL extensions

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

    initGLState();
    initMaterials();
    initGeometry();
    initScene();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
