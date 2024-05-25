#include <iostream>
#include <string>
#include <SFML/Graphics.hpp>

struct Joint {
    float rotation = 0.0f;
    float chain_length = 0.0f;
};

struct Vector2 {
    float x = 0.0f;
    float y = 0.0f;

    Vector2 normalized() const {
        float length = std::sqrt(this->x * this->x + this->y * this->y);
        return { this->x / length, this->y / length };
    }

    void normalize() {
        float length = std::sqrt(this->x * this->x + this->y * this->y);
        this->x /= length;
        this->y /= length;
    }

    float length() {
        return std::sqrt(this->x * this->x + this->y * this->y);
    }
};

float dot(const Vector2& p, const Vector2& q){
    return p.x * q.x + p.y * q.y;
}

const unsigned int joint_count = 4;
const float pi = 3.14159;
const Vector2 origin_pos = {400, 300};
Joint joints[joint_count] = {{0.0f, 50.0f}, {0.0f, 70.0f}, {0.0f, 30.0f}, {0.0f, 15.0f}};
int current_joint_idx = joint_count - 1;
float last_cos_sim = 0.0f;

Vector2 effector_pos(const Joint joints[], const unsigned int joint_count){
    Vector2 end_pos = {0, 0};

    for(auto i = 0; i < joint_count; ++i){
        const auto& j = joints[i];
        end_pos.x += j.chain_length * std::cos(j.rotation);
        end_pos.y += j.chain_length * std::sin(j.rotation);
    }

    return end_pos;
}

int main() {

    sf::RenderWindow window(sf::VideoMode(800, 600), "2D Inverse Kinematics Demo");
    window.setFramerateLimit(960);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)){
            if (event.type == sf::Event::Closed){
                window.close();
            }

            if(event.type == sf::Event::MouseMoved){
                current_joint_idx = joint_count - 1;
            }
        }

        auto mouse_pos_i = sf::Mouse::getPosition(window);
        mouse_pos_i.x -= origin_pos.x;
        mouse_pos_i.y -= origin_pos.y;

        Vector2 mouse_pos = {static_cast<float>(mouse_pos_i.x), mouse_pos_i.y*1.0f};

        // std::cout << mouse_pos.x << " " << mouse_pos.y << std::endl;

        window.clear(sf::Color::White);

        // Perform iteration on current joint
        const Vector2 end_effector_pos = effector_pos(joints, joint_count);
        const Vector2 this_effector_pos = effector_pos(joints, current_joint_idx);
        
        Vector2 end_effector_to_target = {mouse_pos.x - end_effector_pos.x, mouse_pos.y - end_effector_pos.y}; 
        const float original_distance = end_effector_to_target.length();
        end_effector_to_target.normalize();

        Vector2 this_effector_to_target = {mouse_pos.x - this_effector_pos.x, mouse_pos.y - this_effector_pos.y};
        this_effector_to_target.normalize();

        float cos_sim_original = dot(end_effector_to_target, this_effector_to_target);

        // std::cout << original_distance << " " << current_joint_idx << std::endl;

        if(original_distance > 2.0f){
            Joint& joint = joints[current_joint_idx];
            const float joint_original_rot = joint.rotation;

            // Positive case
            joint.rotation = joint_original_rot + pi/256;
            const Vector2 end_effector_positive = effector_pos(joints, joint_count);
            const Vector2 this_effector_positive = effector_pos(joints, current_joint_idx);

            Vector2 end_effector_to_target_positive = {mouse_pos.x - end_effector_positive.x, mouse_pos.y - end_effector_positive.y}; 
            const float positive_distance = end_effector_to_target_positive.length();
            end_effector_to_target_positive.normalize();

            Vector2 this_effector_to_target_positive = {mouse_pos.x - this_effector_positive.x, mouse_pos.y - this_effector_positive.y};
            this_effector_to_target_positive.normalize();

            float cos_sim_positive = dot(end_effector_to_target_positive, this_effector_to_target_positive);

            // Negative case
            joint.rotation = joint_original_rot - pi/256;
            const Vector2 end_effector_negative = effector_pos(joints, joint_count);
            const Vector2 this_effector_negative = effector_pos(joints, current_joint_idx);

            Vector2 end_effector_to_target_negative = {mouse_pos.x - end_effector_negative.x, mouse_pos.y - end_effector_negative.y}; 
            const float negative_distance = end_effector_to_target_negative.length();
            end_effector_to_target_negative.normalize();

            Vector2 this_effector_to_target_negative = {mouse_pos.x - this_effector_negative.x, mouse_pos.y - this_effector_negative.y};
            this_effector_to_target_negative.normalize();

            float cos_sim_negative = dot(end_effector_to_target_negative, this_effector_to_target_negative);

            if(positive_distance < negative_distance){
                joint.rotation = joint_original_rot + pi/256;
            }
            else{
                joint.rotation = joint_original_rot - pi/256;
            }

            if(current_joint_idx == 0){
                current_joint_idx = joint_count - 1;
            }
            else {
                --current_joint_idx;
            }
        }


        // Render the Kinematic Chain
        auto position = origin_pos;
        for(auto i = 0; i < joint_count; ++i){
            const auto& joint = joints[i];

            sf::CircleShape circle(5.0f);
            circle.setFillColor(sf::Color::Black);
            circle.setOrigin({5.0f, 5.0f});
            circle.setPosition(position.x, position.y);
            window.draw(circle);

            sf::RectangleShape line({3, joint.chain_length});
            line.setFillColor(sf::Color::Black);
            line.setOrigin({1.5, joint.chain_length});
            line.setPosition(position.x, position.y);
            line.setRotation(joint.rotation * 180.0f/pi + 90);
            window.draw(line);

            position.x += joint.chain_length * std::cos(joint.rotation);
            position.y += joint.chain_length * std::sin(joint.rotation);

            if(i == (joint_count - 1)){
                sf::CircleShape circle(5.0f);
                circle.setFillColor(sf::Color::Black);
                circle.setOrigin({5.0f, 5.0f});
                circle.setPosition(position.x, position.y);
                window.draw(circle);
            }
        }

        window.display();
    }


    return 0;
}