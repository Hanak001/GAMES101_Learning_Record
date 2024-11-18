// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

std::vector<std::vector<float>> MSAA_step={{0.25,0.25},{0.75,0.25},{0.25,0.25},{0.75,0.75}};

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f v0p={x-_v[0][0],y-_v[0][1],0};
    Eigen::Vector3f v01={_v[1][0]-_v[0][0],_v[1][1]-_v[0][1],0};
    Eigen::Vector3f v1p={x-_v[1][0],y-_v[1][1],0};
    Eigen::Vector3f v12={_v[2][0]-_v[1][0],_v[2][1]-_v[1][1],0};
    Eigen::Vector3f v2p={x-_v[2][0],y-_v[2][1],0};
    Eigen::Vector3f v20={_v[0][0]-_v[2][0],_v[0][1]-_v[2][1],0};

    Eigen::Vector3f vx1=v01.cross(v0p);
    Eigen::Vector3f vx2=v12.cross(v1p);
    Eigen::Vector3f vx3=v20.cross(v2p);

    if((vx1[2]>0&&vx2[2]>0&&vx3[2]>0)||(vx1[2]<0&&vx2[2]<0&&vx3[2]<0)){
        // std::cout<<"x: "<<x<<" y: "<<y<<std::endl;
        return true;
    }
    else{
        return false;
    }
    // return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // Find out the bounding box of current triangle.
    float l=std::min({v[0][0],v[1][0],v[2][0]});
    float r=std::max({v[0][0],v[1][0],v[2][0]});
    float b=std::min({v[0][1],v[1][1],v[2][1]});
    float top=std::max({v[0][1],v[1][1],v[2][1]});

    // iterate through the pixel and find if the current pixel is inside the triangle
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;
    
    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    for(int i=std::ceil(l);i<=std::floor(r);i++){
        for(int j=std::ceil(b);j<=std::floor(r);j++){
            bool judge=false;
            for(int k=0;k<4;k++){
                float x = i+MSAA_step[k][0], y = j+MSAA_step[k][1];
                if(insideTriangle(x,y,t.v)){
                    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    // std::cout<<"X: "<< 2*(x-0.25)<<" Y: "<<2*(y-0.25)<<std::endl;
                    if(z_interpolated>MSAA_depth_buf[get_MSAA_index(2*i+k%2,2*j+k/2)]){
                        judge=true;
                        MSAA_depth_buf[get_MSAA_index(2*i+k%2,2*j+k/2)]=z_interpolated;
                        
                        MSAA_frame_buf[get_MSAA_index(2*i+k%2,2*j+k/2)]=t.getColor();
                    }
                }
            
            }
            if(judge){
                Eigen::Vector3f point = Eigen::Vector3f(i, j, 0);
                Eigen::Vector3f color = (MSAA_frame_buf[get_MSAA_index(2*i,2*j)]+MSAA_frame_buf[get_MSAA_index(2*i+1,2*j)]+MSAA_frame_buf[get_MSAA_index(2*i,2*j+1)]+MSAA_frame_buf[get_MSAA_index(2*i+1,2*j+1)])/4;
                set_pixel(point,color);
            }
            // float x=i,y=j;
            // if(insideTriangle(x,y,t.v)){
            //     auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
            //     float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            //     float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            //     z_interpolated *= w_reciprocal;
            //     if(z_interpolated>depth_buf[get_index(i,j)]){
            //         depth_buf[get_index(i,j)]=z_interpolated;
            //         Vector3f point = { (float)x,(float)y,z_interpolated};
            //         Vector3f color = t.getColor();
            //         //着色
            //         set_pixel(point, color);
            //     }
            // }
            
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(MSAA_frame_buf.begin(), MSAA_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), -std::numeric_limits<float>::infinity());
        std::fill(MSAA_depth_buf.begin(), MSAA_depth_buf.end(), -std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    MSAA_frame_buf.resize(w*h*4);

    depth_buf.resize(w * h);
    MSAA_depth_buf.resize(w*h*4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_MSAA_index(int x, int y)
{
    return (height*2-1-y)*width*2 + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on