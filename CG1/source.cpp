
//! [fullsource]

#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"
#include "OgreApplicationContext.h"
#include "OgreTrays.h"
#include <OgreCamera.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <ois/OIS.h>
#include <thread>
#include <mutex>

using namespace Ogre;
using namespace OgreBites;

std::mutex data_mutex;

struct pointCloud {
    Vector3 position;
    Vector3 color;
};

struct KDNode {
    pointCloud* list;
    int num;
    AxisAlignedBox aabb;
    KDNode* left;
    KDNode* right;

    KDNode() {
        list = nullptr;
        num = 0;
        left = right = nullptr;
    }
};

enum Visibility
{
    NONE,
    PARTIAL,
    FULL
};

Visibility getVisibility(Camera* cam, const AxisAlignedBox& bound);

bool x_cmp(pointCloud p1, pointCloud p2) {
    return p1.position.x < p2.position.x;
}
bool y_cmp(pointCloud p1, pointCloud p2) {
    return p1.position.y < p2.position.y;
}
bool z_cmp(pointCloud p1, pointCloud p2) {
    return p1.position.z < p2.position.z;
}


bool getDataThread(Camera* cam, const KDNode* node, std::vector<std::pair<pointCloud*, int>>* data) {
	Visibility v = getVisibility(cam, node->aabb);
	if (v == FULL) {
        std::lock_guard<std::mutex> guard(data_mutex);
		data->push_back(std::pair<pointCloud*, int>(node->list, node->num));
		std::cout << "full\n";
		return true;
	}
	else if (v == PARTIAL) {
		std::cout << "partial\n";
		if (node->left)
			getDataThread(cam, node->left, data);
		if (node->right)
			getDataThread(cam, node->right, data);
		if (!node->left && !node->right) {
			pointCloud* tmpData = new pointCloud[node->num];
			int tmpNum = 0;
			for (int i = 0; i < node->num; ++i) {
				if (cam->isVisible(node->list[i].position))
					tmpData[tmpNum++] = node->list[i];
			}
			pointCloud* tmpData1 = new pointCloud[tmpNum];
			for (int i = 0; i < tmpNum; ++i) {
				tmpData1[i] = tmpData[i];
			}
			delete[] tmpData;
            std::lock_guard<std::mutex> guard(data_mutex);
            data->push_back(std::pair<pointCloud*, int>(tmpData1, tmpNum));
		}
	}
	return false;
}

class KDTree {
public:
    KDNode* root;
    int maxDepth;
    int maxNumPerBox;
    int tmpCount;
    pointCloud* list;
    int num;
    KDTree() : root(nullptr), maxDepth(10), maxNumPerBox(100), tmpCount(0), list(nullptr), num(0) {};
    KDTree(pointCloud* list, int num, int maxD = 10, int maxN = 100) {
        maxDepth = maxD;
        maxNumPerBox = maxN;
        tmpCount = 0;
        this->num = num;
        this->list = new pointCloud[num];
        memcpy(this->list, list, num * sizeof(pointCloud));
        buildTree(this->list, num);
    }
    ~KDTree() {
        destroyTree(root);
        if (list)
            delete[] list;
    }
    bool buildTree(pointCloud* list, int num) {
        Vector3 minV = Vector3(FLT_MAX, FLT_MAX, FLT_MAX);
        Vector3 maxV = Vector3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        for (int i = 0; i < num; ++i) {
            minV.x = minV.x < list[i].position.x ? minV.x : list[i].position.x;
            minV.y = minV.y < list[i].position.y ? minV.y : list[i].position.y;
            minV.z = minV.z < list[i].position.z ? minV.z : list[i].position.z;
            maxV.x = maxV.x > list[i].position.x ? maxV.x : list[i].position.x;
            maxV.y = maxV.y > list[i].position.y ? maxV.y : list[i].position.y;
            maxV.z = maxV.z > list[i].position.z ? maxV.z : list[i].position.z;
        }
        AxisAlignedBox box(minV, maxV);
        return buildTreeImp(root, list, num, box, 0);
    }
    void destroyTree(KDNode* node) {
        if (!node)
            return;
        destroyTree(node->left);
        destroyTree(node->right);
        delete node;
    }
    std::pair<pointCloud*, int> getData(Camera* cam) {
        std::vector<std::pair<pointCloud*, int>> data;
        //cam->setFOVy(Degree(60));
        //getDataImp(cam, root, data);
        std::thread th1(getDataThread, cam, root->left->left, &data);
        std::thread th2(getDataThread, cam, root->left->right, &data);
        std::thread th3(getDataThread, cam, root->right->left, &data);
        std::thread th4(getDataThread, cam, root->right->right, &data);
        th1.join();
        th2.join();
        th3.join();
        th4.join();
        std::vector<std::pair<pointCloud*, int>>::iterator it;
        int tmpNum = 0;
        for (it = data.begin(); it != data.end(); ++it) {
            tmpNum += it->second;
        }
        pointCloud* rtnData = new pointCloud[tmpNum];
        pointCloud* tmpHead = rtnData;
        for (it = data.begin(); it != data.end(); ++it) {
            memcpy(tmpHead, it->first, it->second * sizeof(pointCloud));
            tmpHead += it->second;
        }
        cam->setFOVy(Degree(45));
        return std::pair<pointCloud*, int>(rtnData, tmpNum);
    }
private:
    bool buildTreeImp(KDNode*& node, pointCloud* list, int num, AxisAlignedBox& box, int depth) {
        node = new KDNode();
        node->list = list;
        node->num = num;
        node->aabb = box;
        tmpCount++;
        if (depth >= maxDepth || num < maxNumPerBox) {
            node->left = node->right = nullptr;
            return true;
        }
        int axis = int(3 * rand() / (RAND_MAX + 1));
        switch (axis)
        {
        case 0:
            std::sort(list, list + num, x_cmp);
            break;
        case 1:
            std::sort(list, list + num, y_cmp);
            break;
        case 2:
            std::sort(list, list + num, z_cmp);
            break;
        default:
            std::sort(list, list + num, x_cmp);
            break;
        }
        int tmpV = list[num / 2].position[axis];
        Vector3 minP = box.getMinimum();
        Vector3 maxP = box.getMaximum();
        Vector3 minMid = minP;
        Vector3 maxMid = maxP;
        minMid[axis] = tmpV - 0.1;
        maxMid[axis] = tmpV + 0.1;
        AxisAlignedBox lBox = AxisAlignedBox(minP, maxMid);
        AxisAlignedBox rBox = AxisAlignedBox(minMid, maxP);
        if (buildTreeImp(node->left, list, num / 2, lBox, depth + 1)
            && buildTreeImp(node->right, list + (num / 2), num - num / 2, rBox, depth + 1))
            return true;
    }
    bool getDataImp(Camera* cam, const KDNode* node, std::vector<std::pair<pointCloud*, int>>& data) {
        Visibility v = getVisibility(cam, node->aabb);
        if (v == FULL) {
            data.push_back(std::pair<pointCloud*, int>(node->list, node->num));
            std::cout << "full\n";
            return true;
        }
        else if (v == PARTIAL) {
            std::cout << "partial\n";
            if (node->left)
                getDataImp(cam, node->left, data);
            if (node->right)
                getDataImp(cam, node->right, data);
            if (!node->left && !node->right) {
                pointCloud* tmpData = new pointCloud[node->num];
                int tmpNum = 0;
                for (int i = 0; i < node->num; ++i) {
                    if (cam->isVisible(node->list[i].position))
                        tmpData[tmpNum++] = node->list[i];
                }
                pointCloud* tmpData1 = new pointCloud[tmpNum];
                for (int i = 0; i < tmpNum; ++i) {
                    tmpData1[i] = tmpData[i];
                }
                delete[] tmpData;
                data.push_back(std::pair<pointCloud*, int>(tmpData1, tmpNum));
            }
        }
        return false;
    }

};

Visibility getVisibility(Camera* cam, const AxisAlignedBox& bound) {
    if (bound.isNull())
        return NONE;
    Vector3 center = bound.getCenter();
    Vector3 halfSize = bound.getHalfSize();

    bool all_inside = true;

    for (uchar plane = 0; plane < 6; ++plane) {
        if (plane == FRUSTUM_PLANE_FAR && cam->getFarClipDistance() == 0)
            continue;

        Plane::Side side = cam->getFrustumPlane(plane).getSide(bound);

        if (side == Plane::NEGATIVE_SIDE)
            return NONE;
        if (side == Plane::BOTH_SIDE)
            all_inside = false;
    }

    if (all_inside)
        return FULL;
    else
        return PARTIAL;
}

class BasicTutorial1
    : public ApplicationContext
    , public InputListener
{
public:
    BasicTutorial1();
    virtual ~BasicTutorial1() {}

    void setup();
    bool frameStarted(const FrameEvent& evt) {
        ApplicationContext::frameStarted(evt);
        count=(count + 1)%60;
        if (count == 0) {
            time = getRoot()->getTimer()->getMilliseconds();
        }

        m_Mouse->capture();
        OIS::MouseState mousePos = m_Mouse->getMouseState();
        
        x += mousePos.X.rel; y += mousePos.Y.rel;
        float rotateSpeed = 90/300.0;
        camNode->resetOrientation(); camNode->setDirection(front, Node::TS_WORLD);
        camNode->yaw(-Degree(rotateSpeed * x));
        camNode->pitch(-Degree(rotateSpeed * y));

        m_Keyboard->capture();
        if (m_Keyboard->isKeyDown(OIS::KC_ESCAPE)) {
            getRoot()->queueEndRendering();
        }

        float moveSpeed = 50; Vector3 right = front.crossProduct(up);
        if (m_Keyboard->isKeyDown(OIS::KC_A)) {
            camNode->setPosition(camNode->getPosition() - moveSpeed * evt.timeSinceLastFrame * right);
        }
        if (m_Keyboard->isKeyDown(OIS::KC_D)) {
            camNode->setPosition(camNode->getPosition() + moveSpeed * evt.timeSinceLastFrame * right);
        }
        if (m_Keyboard->isKeyDown(OIS::KC_Q)) {
            camNode->setPosition(camNode->getPosition() + moveSpeed * evt.timeSinceLastFrame * up);
        }
        if (m_Keyboard->isKeyDown(OIS::KC_E)) {
            camNode->setPosition(camNode->getPosition() - moveSpeed * evt.timeSinceLastFrame * up);
        }
        if (m_Keyboard->isKeyDown(OIS::KC_W)) {
            camNode->setPosition(camNode->getPosition() + moveSpeed * evt.timeSinceLastFrame * front);
        }
        if (m_Keyboard->isKeyDown(OIS::KC_S)) {
            camNode->setPosition(camNode->getPosition() - moveSpeed * evt.timeSinceLastFrame * front);
        }

        if (m_Keyboard->isKeyDown(OIS::KC_C)) {
            std::pair<pointCloud*, int> p = kdTree->getData(cam);
            // std::cout << "finished\n";
             //printf("count: %d\n", p.second);
            clouds->clear();
            clouds->begin("largePoint", RenderOperation::OT_POINT_LIST);
            for (int i = 0; i < p.second; ++i) {
                clouds->position(p.first[i].position);
                clouds->colour(p.first[i].color.x / 255.0, p.first[i].color.y / 255.0, p.first[i].color.z / 255.0);
            }
            clouds->end();
            ogreNode->detachAllObjects();
            ogreNode->attachObject(clouds);
            panel->setParamValue("pointNum", std::to_string(p.second));
        }

        return true;
    };
    bool frameEnded(const FrameEvent& evt) {
        ApplicationContext::frameEnded(evt);
        if (count == 59) {
            float renderTime = (getRoot()->getTimer()->getMilliseconds() - time) / 60.0;
            panel->setParamValue("render time per frame", "\n"+std::to_string(renderTime) + "ms");
        }
        return true;
    }
private:
    bool flag = true;
    SceneNode* ogreNode;
    float time=0;
    ParamsPanel* panel;
    int count = 0;
    OIS::Mouse* m_Mouse;
    OIS::Keyboard* m_Keyboard;
    SceneNode* camNode;
    Camera* cam;
    int x = 0, y = 0;
    Vector3 front, up;
    ManualObject* clouds;
    KDTree* kdTree;
};


BasicTutorial1::BasicTutorial1()
    : ApplicationContext("OgreTutorialApp")
{
}


void BasicTutorial1::setup()
{
    // do not forget to call the base first
    ApplicationContext::setup();
    addInputListener(this);

    // get a pointer to the already created root
    Root* root = getRoot();
    SceneManager* scnMgr = root->createSceneManager();
    RTShader::ShaderGenerator* shadergen = RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);

    scnMgr->addRenderQueueListener(getOverlaySystem());
    TrayManager* mTrayMgr = new TrayManager("test", getRenderWindow());
    addInputListener(mTrayMgr);

    //! [camera]
    camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    cam = scnMgr->createCamera("myCam");
    cam->setNearClipDistance(5); // specific to this sample
    cam->setAutoAspectRatio(true);
    cam->setFOVy(Degree(45));
    camNode->attachObject(cam); Vector3 camPos(0, -200, 0);
    camNode->setPosition(camPos);
    camNode->setDirection(Vector3(0, 1, 0), Node::TS_WORLD);
    camNode->setInitialState();
    front = Vector3(0, 1, 0); up = Vector3(0, 0, 1);
   
    // and tell it to render into the main window
    Viewport* vp = getRenderWindow()->addViewport(cam);
    //! [camera]
    vp->setBackgroundColour(ColourValue::Black);
    
    //ui
    mTrayMgr->showFrameStats(TL_TOPLEFT); std::vector<std::string> a;
    a.push_back("pointNum"); a.push_back("render time per frame"); a.push_back("");
    panel = mTrayMgr->createParamsPanel(TL_TOPLEFT, "time", 200, a);

    //cloud
    /*ogreNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    ManualObject* pointCloud = scnMgr->createManualObject();
    pointCloud->begin("BaseWhiteNoLighting", RenderOperation::OT_POINT_LIST);
    std::ifstream file("Pasha_guard_head400K.txt");
    std::string tmp;
    std::getline(file, tmp); std::getline(file, tmp);
    int lineNumber = std::atoi(tmp.c_str()); p->setParamValue("pointNum", std::to_string(lineNumber));
    while (std::getline(file, tmp)) {
        float ntmp[6];
        for (int i = 0; i < 5; i++) {
            int pos = tmp.find(' ');
            ntmp[i] = std::atof(tmp.substr(0, pos).c_str());
            tmp = tmp.substr(pos + 1, tmp.size());
        }
        ntmp[5] = std::atof(tmp.c_str());
        pointCloud->position(Vector3(ntmp[0], ntmp[1], ntmp[2])); 
        pointCloud->colour(ntmp[3]/255, ntmp[4]/255, ntmp[5]/255);
    }
    pointCloud->end();
    ogreNode->attachObject(pointCloud);*/
    char line[1000];
    std::string fileName = "Pasha_guard_head400K.txt";
    std::ifstream fIn(fileName);
    if (!fIn.is_open()) {
        std::cerr << "fail to open model file: " << fileName << std::endl;
        return;
    }
    fIn.getline(line, 1000);
    std::cout << line << std::endl;
    fIn.getline(line, 1000);
    int count = atoi(line);
    if (count <= 0) {
        std::cerr << "no point data!\n";
        return;
    }
    pointCloud* data = new pointCloud[count];
    int i = 0;
    Vector3 boundingMin = Vector3(FLT_MAX);
    Vector3 boundingMax = Vector3(-FLT_MAX);
    while (i < count) {
        fIn.getline(line, 1000);
        std::stringstream ss(line);
        ss >> data[i].position[0] >> data[i].position[1] >> data[i].position[2];
        ss >> data[i].color[0] >> data[i].color[1] >> data[i].color[2];
        for (int j = 0; j < 3; ++j) {
            boundingMax[j] = boundingMax[j] > data[i].position[j] ? boundingMax[j] : data[i].position[j];
            boundingMin[j] = boundingMin[j] < data[i].position[j] ? boundingMin[j] : data[i].position[j];
        }
        ++i;
    }
    std::cout << boundingMax << boundingMin;
    for (int i = 0; i < 3; ++i) {
        boundingMax[i] += 1;
        boundingMin[i] -= 1;
    }
    AxisAlignedBox box(boundingMin, boundingMax);
    kdTree = new KDTree(data, count);
    clouds = scnMgr->createManualObject();
    ogreNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    std::pair<pointCloud*, int> p = kdTree->getData(cam);
    // std::cout << "finished\n";
     //printf("count: %d\n", p.second);
    clouds->clear();
    clouds->begin("largePoint", RenderOperation::OT_POINT_LIST);
    for (int i = 0; i < p.second; ++i) {
        clouds->position(p.first[i].position);
        clouds->colour(p.first[i].color.x / 255.0, p.first[i].color.y / 255.0, p.first[i].color.z / 255.0);
    }
    clouds->end();
    ogreNode->detachAllObjects();
    ogreNode->attachObject(clouds);
    panel->setParamValue("pointNum", std::to_string(p.second));


    OIS::ParamList pl;
    size_t windowHnd = 0;
    std::ostringstream windowHndStr;
    getRenderWindow()->getCustomAttribute("WINDOW", &windowHnd);
    windowHndStr << windowHnd;
    pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
    //pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_FOREGROUND")));
    //pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_NONEXCLUSIVE")));
    //pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_FOREGROUND")));
    //pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_NONEXCLUSIVE")));
    OIS::InputManager* m_InputManager = OIS::InputManager::createInputSystem(pl);
    m_Mouse = static_cast<OIS::Mouse*>(m_InputManager->createInputObject(OIS::OISMouse, false));
    m_Keyboard = static_cast<OIS::Keyboard*>(m_InputManager->createInputObject(OIS::OISKeyboard, false));
    unsigned int width, height, depth;
    int top, left;
    getRenderWindow()->getMetrics(width, height, depth, left, top);
    const OIS::MouseState& ms = m_Mouse->getMouseState();
    ms.width = width;
    ms.height = height;
}

int main(int argc, char** argv)
{
    try
    {
        BasicTutorial1 app;
        app.initApp();
        app.getRoot()->startRendering();
        app.closeApp();
    }
    catch (const std::exception & e)
    {
        std::cerr << "Error occurred during execution: " << e.what() << '\n';
        return 1;
    }

    return 0;
}

//! [fullsource]
