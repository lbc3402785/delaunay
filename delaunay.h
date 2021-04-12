#ifndef DELAUNAY_H
#define DELAUNAY_H
#include <vector>
#include <algorithm>
#include <iostream>
#include <utility>
#include <assert.h>
//#define SHOW
#ifdef SHOW
#include <opencv2/opencv.hpp>
#endif
template <typename T,typename Point2T>
class Delaunay
{
private:
    T EPSILON;
#ifdef SHOW
    cv::Mat test;
#endif
    class Circumcircle{
    public:
        Circumcircle(Point2T& center,T& radius2,size_t i,size_t j,size_t k):center(center),radius2(radius2)
        {
            tri={i,j,k};
        }
        T radius2;
        Point2T center;
        std::array<size_t, 3> tri;
        friend std::ostream &operator<<(std::ostream &out, Circumcircle &x){
            out<<"Center:"<<x.center<<std::endl;
            out<<"radius:"<<std::sqrt(x.radius2)<<std::endl;
            out<<"tri:"<<x.tri[0]<<","<<x.tri[1]<<","<<x.tri[2]<<std::endl;
            return out;
        }
    };
    std::vector<Point2T> points;
    std::vector<size_t> indices;
    void dedup(std::vector<std::pair<size_t,size_t>>& edges);
    std::array<Point2T,3> superTriangle()
    {
        T xmin=points[0].x;
        T xmax=points[0].x;
        T ymin=points[0].y;
        T ymax=points[0].y;
        for(size_t k=1;k<points.size();k++)
        {
            T x=points[k].x;
            T y=points[k].y;
            if(x<xmin){
                xmin=x;
            }else if(x>xmax){
                xmax=x;
            }
            if(y<ymin){
                ymin=y;
            }else if(y>ymax){
                ymax=y;
            }
        }
        T width=xmax-xmin;
        width/=2.0;
        T height=ymax-ymin;
        Point2T left(xmin-width-2,ymax+1);
        Point2T top(xmin+width,ymin-height);
        Point2T right(xmax+width+2,ymax+1);
        return {left,top,right};
    }
    Circumcircle circumcircle(size_t i,size_t j,size_t k){
        T x1=points[i].x;
        T y1=points[i].y;
        T x2=points[j].x;
        T y2=points[j].y;
        T x3=points[k].x;
        T y3=points[k].y;
        T fabsy1y2=std::fabs(y1-y2);
        T fabsy2y3=std::fabs(y2-y3);
        T xc,yc,m1,m2,mx1,mx2,my1,my2,dx,dy;

        if(fabsy1y2<EPSILON&&fabsy2y3<EPSILON){
            std::cerr<<"3 points stand in one line!"<<std::endl;
            exit(EXIT_FAILURE);
        }
        if(fabsy1y2<EPSILON){
            m2=-(x3-x2)/(y3-y2);
            mx2=(x2+x3)/2.0;
            my2=(y2+y3)/2.0;
            xc=(x2+x1)/2.0;
            yc=m2*(xc-mx2)+my2;
        }else if(fabsy2y3<EPSILON){
            m1=-(x2-x1)/(y2-y1);
            mx1=(x1+x2)/2.0;
            my1=(y1+y2)/2.0;
            xc=(x3+x2)/2.0;
            yc=m1*(xc-mx1)+my1;
        }else{
            m1 = -((x2 - x1) / (y2 - y1));
            m2 = -((x3 - x2) / (y3 - y2));
            mx1 = (x1 + x2) / 2.0;
            mx2 = (x2 + x3) / 2.0;
            my1 = (y1 + y2) / 2.0;
            my2 = (y2 + y3) / 2.0;
            xc = (m1 * mx1 - m2 * mx2 + my2 - my1) / (m1 - m2);
            yc = (fabsy1y2 > fabsy2y3) ?
                        m1 * (xc - mx1) + my1 :
                        m2 * (xc - mx2) + my2;
        }
        dx=x2-xc;
        dy=y2-yc;
        Point2T center(xc,yc);
        T radius2=dx*dx+dy*dy;
        return Circumcircle(center,radius2,i,j,k);
    }
public:
    Delaunay(T*data,size_t size)
    {
        std::vector<T>vertices(data,data+size);
        assert(vertices.size()>6&&vertices.size()%2==0);
        points.resize(vertices.size()/2);
        indices.resize(vertices.size()/2);
        for(size_t k=0;k<vertices.size()/2;k++)
        {
            Point2T p(vertices[2*k+0],vertices[2*k+1]);
            points[k]=p;
            indices[k]=k;
        }
        init();
    }
    Delaunay(std::vector<T>& vertices)
    {
        assert(vertices.size()>6&&vertices.size()%2==0);
        points.resize(vertices.size()/2);
        indices.resize(vertices.size()/2);
        for(size_t k=0;k<vertices.size()/2;k++)
        {
            Point2T p(vertices[2*k+0],vertices[2*k+1]);
            points[k]=p;
            indices[k]=k;
        }
        init();
    }
    Delaunay(std::vector<Point2T>& points)
        :points(points)
    {
        assert(points.size()>3);
        indices.resize(points.size());
        for(size_t k=0;k<points.size();k++)
        {
            indices[k]=k;
        }
        init();
    }
    void init(){
        std::sort(indices.begin(),indices.end(),[&](const size_t &i,const size_t&j){
            return points[i].x<=points[j].x;
        });
        EPSILON =std::numeric_limits<T>::epsilon();
#ifdef SHOW
        test=cv::Mat::zeros(500,500,CV_8UC3);
#endif
    }
#ifdef SHOW
    bool check(cv::Mat& img,Point2T& p)
    {
        if(p.x>=0&&p.x<=img.cols&&p.y>=0&&p.y<=img.rows){
            return true;
        }
        return false;
    }
    void show(cv::Mat& input,std::vector<Circumcircle>& circles,std::vector<std::pair<size_t,size_t>>&edges)
    {
        using namespace cv;
        cv::Mat img=input.clone();
        cv::Mat edgesMap=input.clone();
        for (size_t i =0;i<circles.size(); i++){
            size_t j=circles[i].tri[0];
            size_t k=circles[i].tri[1];
            size_t l=circles[i].tri[2];
            if(check(img,points[j])&&check(img,points[k])){
                cv::line(img,Point(points[j].x,points[j].y),Point(points[k].x,points[k].y),cv::Scalar(0,0,255),1,CV_AA);
            }
            if(check(img,points[k])&&check(img,points[l])){
                cv::line(img,Point(points[k].x,points[k].y),Point(points[l].x,points[l].y),cv::Scalar(0,0,255),1,CV_AA);
            }
            if(check(img,points[l])&&check(img,points[j])){
                cv::line(img,Point(points[l].x,points[l].y),Point(points[j].x,points[j].y),cv::Scalar(0,0,255),1,CV_AA);
            }
        }
        for(size_t i=0;i<points.size()-3;i++){
            cv::circle(img,Point(points[i].x,points[i].y),1,Scalar(255,0,0),-1,CV_AA);
        }
        for(size_t i=0;i<edges.size();i++){
            size_t j=edges[i].first;
            size_t k=edges[i].second;
            cv::line(edgesMap,Point(points[j].x,points[j].y),Point(points[k].x,points[k].y),cv::Scalar(0,255,0),1,CV_AA);
        }
        cv::imshow("process:",img);
        cv::imshow("edge:",edgesMap);
        cv::waitKey(0);
    }
#endif
    std::vector<std::array<size_t, 3>> triangle()
    {
        size_t n=points.size();
        int i,j;
        size_t current;
        T dx,dy;
        std::vector<Circumcircle> open;
        std::vector<Circumcircle> closed;
        std::vector<std::pair<size_t,size_t>> buffer;
        std::array<Point2T,3> superTri=superTriangle();
        points.push_back(superTri[0]);
        points.push_back(superTri[1]);
        points.push_back(superTri[2]);
        open.push_back(circumcircle(n,n+1,n+2));
#ifdef SHOW
        show(test,open,buffer);
#endif
        //starting loop from smallest x of vertex
        for(i=0;i<indices.size();i++){
            current=indices[i];
            for(auto it=open.begin();it!=open.end();){
                dx = points[current].x - it->center.x;
                //On the right side of the circumcircle
                if (dx > 0 && dx * dx > it->radius2) {
                    closed.push_back(*it);
                    it=open.erase(it);//delete index
#ifdef SHOW
                    show(test,open,buffer);
#endif
                    continue;
                }
                //skip this point if it is outside this circumcircle
                dy = points[current].y - it->center.y;
                if (dx * dx + dy * dy - it->radius2 > EPSILON){
                    it++;
                    continue;
                }
                buffer.push_back(std::pair<size_t,size_t>(it->tri[0],it->tri[1]));
                buffer.push_back(std::pair<size_t,size_t>(it->tri[1],it->tri[2]));
                buffer.push_back(std::pair<size_t,size_t>(it->tri[2],it->tri[0]));
                it=open.erase(it);//delete index
#ifdef SHOW
                show(test,open,buffer);
#endif           
            }
            dedup(buffer);
            for (j=0;j<buffer.size();j++){
                open.push_back(circumcircle(buffer[j].first, buffer[j].second, current));
            }
            buffer.clear();
#ifdef SHOW
            show(test,open,buffer);
#endif
        }
        for (i =0;i<open.size();i++)
            closed.push_back(open[i]);
        std::vector<std::array<size_t, 3>> result;
        for (i =0;i<closed.size(); i++){
            if (closed[i].tri[0] < n && closed[i].tri[1] < n && closed[i].tri[2] < n)//deleting triangles that are relative to supertriangle
            {
                result.push_back(closed[i].tri);
            }
        }
        return result;
    }
};
template<typename T, typename Point2T>
void Delaunay<T,Point2T>::dedup(std::vector<std::pair<size_t, size_t> > &edges)
{
    if(edges.size()==0)return;
    size_t i,j,a,b,m,n;
    std::vector<bool> isDelete(edges.size(),false);
    for(j=0;j<edges.size()-1;j++){
        std::pair<size_t,size_t>& edge=edges[j];
        b=edge.first;
        a=edge.second;
        for(i=j+1;i<edges.size();i++){
            std::pair<size_t,size_t>& current=edges[i];
            n=current.first;
            m=current.second;
            if((a==m&&b==n)||(a==n&&b==m)){
                isDelete[j]=isDelete[i]=true;
            }
        }
    }
    std::vector<std::pair<size_t, size_t> > tmp;
    for(j=0;j<edges.size();j++){
        if(!isDelete[j]){
            tmp.push_back(edges[j]);
        }
    }
    edges.swap(tmp);
}
#endif // DELAUNAY_H


