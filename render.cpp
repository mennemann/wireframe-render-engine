#define WIDTH 1440
#define HEIGHT 920
#define FOCAL_LENGTH 1500
#define THREAD_C 10


#include <iostream>
#include <list>
#include <cmath>

#include <chrono>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


class Vertex {
    public:
        static std::vector<Vertex*> vertices;
        float x,y,z;
        int px, py;
        
        Vertex(int x, int y, int z) {
            this->x = x;
            this->y = y;
            this->z = z;
            vertices.push_back(this);
        }

        void project(){
            this->px = ( (FOCAL_LENGTH * this->x) / (FOCAL_LENGTH + this->z) ) + (WIDTH/2);
            this->py = ( (FOCAL_LENGTH * this->y) / (FOCAL_LENGTH + this->z) ) + (HEIGHT/2);
        }
};

std::vector<Vertex*>Vertex::vertices;

struct Edge {
    Vertex* v1;
    Vertex* v2;
};

void gen_img(double(*img)[WIDTH], auto edges) {
    memset(img, 0, sizeof(double) * WIDTH * HEIGHT);
    for (auto const& v: Vertex::vertices) {
        v->project();
        img[v->py][v->px] = 1;
    }

    for (auto const& e: *edges) {
        float m;
        bool f = false;

        int y_diff = e.v2->py - e.v1->py;
        int x_diff = e.v2->px - e.v1->px;
        
        if (x_diff != 0) {
            m = (float)y_diff / (float)x_diff;
            if (-1 > m || m > 1) f = true;
        } else f = true;
        
        if (f && (y_diff != 0)) {
            m = (float)x_diff / (float)y_diff;
            if (-1 > m || m > 1) exit(-1);
        }

        int start_p, stop_p, b;
        if(!f) {
            start_p = std::min(e.v1->px, e.v2->px);
            stop_p = std::max(e.v1->px, e.v2->px);
            b = e.v2->py - m * e.v2->px;
        } else {
            start_p = std::min(e.v1->py, e.v2->py);
            stop_p = std::max(e.v1->py, e.v2->py);
            b = e.v2->px - m * e.v2->py;
        }

        for (int i = start_p; i < stop_p; i++) { 
            int xy = m*i+b;
            if (!f) img[xy][i] = 1;
            if (f) img[i][xy] = 1;
        }
    }
}

void def_wireframe(std::vector<Edge>* edges) {
    auto a = new Vertex(-250,-250,50);
    auto b = new Vertex(-250,250,50);
    auto c = new Vertex(250,-250,50);
    auto d = new Vertex(250,250,50);
    auto e = new Vertex(-250,-250,550);
    auto f = new Vertex(-250,250,550);
    auto g = new Vertex(250,-250,550);
    auto h = new Vertex(250,250,550);

    edges->insert(edges->end(), {
        Edge {a,c},Edge {b,d},Edge {a,b},Edge {c,d},
        Edge {e,g},Edge {f,h},Edge {e,f},Edge {g,h},
        Edge {a,e},Edge {c,g},Edge {b,f},Edge {d,h}
    });
}

void def_wireframe2(std::vector<Edge>* edges) {
    float R,r;

    R=100;
    r=50;
    int c = 0;
    float theta = 0;
    float factor = 2.5;

    int ring_off = 0;
    while (theta < 2*M_PI) {
        float gamma = 0;
        ring_off = 0;
        while (gamma < 2*M_PI) {
            new Vertex((R+(r*cos(theta)))*cos(gamma)*factor, (R+(r*cos(theta)))*sin(gamma)*factor, factor*r*sin(theta)+300);
            gamma += 0.01*M_PI;
            c++;
        }
        ring_off++;
        theta += 0.01*M_PI;
    }
    
    
    for (int i = 0; i < Vertex::vertices.size()-2; i++){
        edges->push_back(Edge {Vertex::vertices[i], Vertex::vertices[i+2]});
    }

    std::cout<<c<<std::endl;
}

const float alpha = 0.01;
const float beta = 0.01;
const float theta = 0.01;

const float fac_x11 = cos(beta)*cos(theta);
const float fac_x12 = sin(alpha)*sin(beta)*cos(theta)-cos(alpha)*sin(theta);
const float fac_x13 = cos(alpha)*sin(beta)*cos(theta) + sin(alpha)*sin(theta);

const float fac_x21 = cos(beta)*sin(theta);
const float fac_x22 = sin(alpha)*sin(beta)*sin(theta)+cos(alpha)*cos(theta);
const float fac_x23 = cos(alpha)*sin(beta)*sin(theta) - sin(alpha)*cos(theta);

const float fac_x31 = -1*sin(beta);
const float fac_x32 = sin(alpha)*cos(beta);
const float fac_x33 = cos(alpha)*cos(beta);


void on_frame(int frame_c, Vertex* v) {
    

    v->z -= 275;

    float nx = (fac_x11)*(float)v->x + (fac_x12)*(float)v->y + (fac_x13)*(float)v->z;
    float ny = (fac_x21)*(float)v->x + (fac_x22)*(float)v->y + (fac_x23)*(float)v->z;
    float nz = (fac_x31)*(float)v->x + (fac_x32)*(float)v->y + (fac_x33)*(float)v->z;

    v->x = nx;
    v->y = ny;
    v->z = nz;


    v->z += 275;
}

void on_frame_thread(int tn, int tc) {
    int block_len = (Vertex::vertices.size() / tc);
    int offset = block_len * tn;
    for (int i = offset; i < offset+block_len; i++) {
        on_frame(0, Vertex::vertices[i]);
    }
}

int main(int argc, char** argv)
{
    double (*img)[WIDTH] = new double[HEIGHT][WIDTH];
    
    std::vector<Edge> edges;
    def_wireframe2(&edges);

    int frame_c = 0;
    while (1){
        gen_img(img, &edges);
        cv::Mat img_mat = cv::Mat(HEIGHT, WIDTH, CV_64F, img);
        cv::namedWindow("Test", cv::WINDOW_AUTOSIZE);
        cv::imshow("Test", img_mat);
    
        
        std::vector<std::thread> threads;
        for (int i=0; i<THREAD_C; i++){
            threads.push_back(std::thread(on_frame_thread, i, THREAD_C));
        }
        for (auto& th : threads) th.join();

        frame_c++;
        std::cout<<frame_c<<std::endl;

        if (cv::waitKey(1) == 0) break;
    }
    return 0;
}
