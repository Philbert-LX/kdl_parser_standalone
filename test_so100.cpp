#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <vector>
#include <memory>
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/tree.hpp"
#include "kdl/segment.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainiksolverpos_nr.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "urdf_model/model.h"
#include "urdf_parser/urdf_parser.h"

// 递归打印树结构
void printTree(const KDL::SegmentMap::const_iterator& segment, int depth = 0)
{
    const std::string indent(depth * 2, ' ');
    std::cout << indent << "├─ " << segment->first << std::endl;
    
    // 打印关节信息
    const KDL::Joint& joint = segment->second.segment.getJoint();
    if (joint.getType() != KDL::Joint::None) {
        std::cout << indent << "  └─ Joint: " << joint.getName() 
                  << " (Type: ";
        switch (joint.getType()) {
            case KDL::Joint::RotAxis:
                std::cout << "Rotational";
                break;
            case KDL::Joint::TransAxis:
                std::cout << "Prismatic";
                break;
            case KDL::Joint::None:
                std::cout << "Fixed";
                break;
            default:
                std::cout << "Unknown";
                break;
        }
        std::cout << ")" << std::endl;
    }
    
    // 递归打印子节点
    for (const auto& child : segment->second.children) {
        printTree(child, depth + 1);
    }
}

/**
 * 从URDF文件加载KDL树结构
 * 
 * @param urdf_file_path URDF文件的路径
 * @param tree 输出参数，KDL树对象
 * @return 加载是否成功
 */
bool load_urdf_to_kdl(const std::string& urdf_file_path, KDL::Tree& tree)
{
    // 检查文件是否存在
    std::ifstream file_check(urdf_file_path);
    if (!file_check.good()) {
        std::cout << "urdf not exists - " << urdf_file_path << std::endl;
        return false;
    }
    file_check.close();
    
    // 方法1: 使用 treeFromFile
    try {
        if (kdl_parser::treeFromFile(urdf_file_path, tree)) {
            std::cout << "load sucess: " << urdf_file_path << std::endl;
            std::cout << "chain has " << tree.getNrOfSegments() << " segs and " 
                      << tree.getNrOfJoints() << " joints" << std::endl;
            return true;
        }
    } catch (const std::exception& e) {
        std::cout << "methods1 failed: " << e.what() << std::endl;
    }
    
    // 方法2: 使用二进制模式读取文件并移除编码声明
    try {
        std::ifstream file(urdf_file_path, std::ios::binary);
        if (!file.good()) {
            std::cout << "method2 failed: cannot open file" << std::endl;
        } else {
            // 读取二进制数据
            file.seekg(0, std::ios::end);
            size_t file_size = file.tellg();
            file.seekg(0, std::ios::beg);
            
            std::string xml_content(file_size, '\0');
            file.read(&xml_content[0], file_size);
            
            // 尝试解码为字符串（UTF-8）
            std::string xml_str;
            try {
                xml_str = xml_content;
            } catch (...) {
                // 如果UTF-8失败，尝试latin-1（在C++中通常不需要特殊处理）
                xml_str = xml_content;
            }
            
            // 移除XML编码声明
            size_t xml_decl_pos = xml_str.find("<?xml");
            if (xml_decl_pos != std::string::npos) {
                size_t end_pos = xml_str.find("?>", xml_decl_pos);
                if (end_pos != std::string::npos) {
                    xml_str = xml_str.substr(end_pos + 2);
                }
            }
            
            // 从字符串创建URDF模型
            urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDF(xml_str);
            if (robot_model) {
                // 从URDF模型创建KDL树
                if (kdl_parser::treeFromUrdfModel(*robot_model, tree)) {
                    std::cout << "method2 sucess: " << urdf_file_path << std::endl;
                    std::cout << "chain has " << tree.getNrOfSegments() << " segs and " 
                              << tree.getNrOfJoints() << " joints" << std::endl;
                    return true;
                } else {
                    std::cout << "method2 failed" << std::endl;
                }
            }
        }
    } catch (const std::exception& e) {
        std::cout << "method2 failed: " << e.what() << std::endl;
    }
    
    // 方法3: 直接从文件解析URDF模型
    try {
        urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDFFile(urdf_file_path);
        if (robot_model) {
            if (kdl_parser::treeFromUrdfModel(*robot_model, tree)) {
                std::cout << "method3 sucess: " << urdf_file_path << std::endl;
                std::cout << "chain has " << tree.getNrOfSegments() << " segs and " 
                          << tree.getNrOfJoints() << " joints" << std::endl;
                return true;
            } else {
                std::cout << "method3 failed" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cout << "method3 failed: " << e.what() << std::endl;
    }
    
    std::cout << "can not load: " << urdf_file_path << std::endl;
    return false;
}

/**
 * 从KDL树创建运动链
 * 
 * @param tree KDL树对象
 * @param base_link 基链接名称
 * @param tip_link 末端执行器链接名称
 * @param chain 输出参数，KDL链对象
 * @return 创建是否成功
 */
bool create_chain_from_tree(const KDL::Tree& tree, const std::string& base_link, 
                            const std::string& tip_link, KDL::Chain& chain)
{
    try {
        if (tree.getChain(base_link, tip_link, chain)) {
            std::cout << "create chain from " << base_link << " to " << tip_link << std::endl;
            std::cout << "chain has " << chain.getNrOfSegments() << " segs and " 
                      << chain.getNrOfJoints() << " joints" << std::endl;
            return true;
        } else {
            std::cout << "create chain failed: cannot get chain from tree" << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cout << "create chain failed: " << e.what() << std::endl;
        return false;
    }
}

/**
 * 设置正逆运动学求解器
 * 
 * @param chain 运动链对象
 * @param fk_solver 输出参数，正向运动学求解器
 * @param ik_vel_solver 输出参数，逆速度运动学求解器
 * @param ik_pos_solver 输出参数，逆位置运动学求解器
 */
void setup_kinematics(const KDL::Chain& chain,
                     std::unique_ptr<KDL::ChainFkSolverPos_recursive>& fk_solver,
                     std::unique_ptr<KDL::ChainIkSolverVel_pinv>& ik_vel_solver,
                     std::unique_ptr<KDL::ChainIkSolverPos_NR>& ik_pos_solver)
{
    fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
    ik_vel_solver = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain);
    
    const int max_iterations = 100;
    const double eps = 1e-6;
    ik_pos_solver = std::make_unique<KDL::ChainIkSolverPos_NR>(
        chain, *fk_solver, *ik_vel_solver, max_iterations, eps
    );
}

/**
 * 执行正向运动学计算
 * 
 * @param fk_solver 正向运动学求解器
 * @param joint_positions 关节位置列表
 * @param frame 输出参数，KDL Frame对象，表示末端执行器的位姿
 * @return 计算是否成功
 */
bool perform_forward_kinematics(KDL::ChainFkSolverPos_recursive& fk_solver,
                                const std::vector<double>& joint_positions,
                                KDL::Frame& frame)
{
    int num_joints = joint_positions.size();
    KDL::JntArray q(num_joints);
    for (int i = 0; i < num_joints; ++i) {
        q(i) = joint_positions[i];
    }
    
    int status = fk_solver.JntToCart(q, frame);
    
    if (status >= 0) {
        KDL::Vector pos = frame.p;
        double x, y, z, w;
        frame.M.GetQuaternion(x, y, z, w);
        
        std::cout << "fk sucess:" << std::endl;
        std::cout << "pos: [" << std::fixed << std::setprecision(6)
                  << pos.x() << ", " << pos.y() << ", " << pos.z() << "]" << std::endl;
        std::cout << "quat: [" << std::fixed << std::setprecision(6)
                  << x << ", " << y << ", " << z << ", " << w << "]" << std::endl;
        return true;
    } else {
        std::cout << "fk failed" << std::endl;
        return false;
    }
}

/**
 * 执行逆运动学计算
 * 
 * @param ik_pos_solver 逆位置运动学求解器
 * @param chain 运动链对象
 * @param target_frame 目标位姿 (KDL Frame)
 * @param initial_joints 初始关节配置 (可选，如果为空则使用零配置)
 * @param joint_positions 输出参数，关节位置列表
 * @return 计算是否成功
 */
bool perform_inverse_kinematics(KDL::ChainIkSolverPos_NR& ik_pos_solver,
                                const KDL::Chain& chain,
                                const KDL::Frame& target_frame,
                                const std::vector<double>& initial_joints,
                                std::vector<double>& joint_positions)
{
    int num_joints = chain.getNrOfJoints();
    KDL::JntArray q_init(num_joints);
    
    if (initial_joints.empty() || initial_joints.size() != static_cast<size_t>(num_joints)) {
        // 使用零配置
        for (int i = 0; i < num_joints; ++i) {
            q_init(i) = 0.0;
        }
    } else {
        for (int i = 0; i < num_joints; ++i) {
            q_init(i) = initial_joints[i];
        }
    }
    
    KDL::JntArray q_out(num_joints);
    int status = ik_pos_solver.CartToJnt(q_init, target_frame, q_out);
    
    if (status >= 0) {
        joint_positions.clear();
        joint_positions.reserve(num_joints);
        for (int i = 0; i < num_joints; ++i) {
            joint_positions.push_back(q_out(i));
        }
        
        std::cout << "ik sucess:" << std::endl;
        std::cout << "Joints: [";
        for (size_t i = 0; i < joint_positions.size(); ++i) {
            std::cout << std::fixed << std::setprecision(6) << joint_positions[i];
            if (i < joint_positions.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        return true;
    } else {
        std::cout << "ik failed" << std::endl;
        return false;
    }
}

int main(int argc, char** argv)
{
    // URDF 文件路径（可以通过命令行参数指定）
    std::string urdf_file = "so100.urdf";
    if (argc > 1) {
        urdf_file = argv[1];
    }
    
    // 加载 URDF 到 KDL 树
    KDL::Tree tree;
    bool success = load_urdf_to_kdl(urdf_file, tree);
    if (!success) {
        std::cout << std::endl;
        std::cout << "Press Enter to exit..." << std::endl;
        std::cin.get();
        return 1;
    }
    
    // 创建运动链
    std::string base_link = "Base";
    std::string tip_link = "Moving Jaw";
    
    if (argc > 2) {
        base_link = argv[2];
    }
    if (argc > 3) {
        tip_link = argv[3];
    }
    
    KDL::Chain chain;
    if (!create_chain_from_tree(tree, base_link, tip_link, chain)) {
        std::cout << std::endl;
        std::cout << "Press Enter to exit..." << std::endl;
        std::cin.get();
        return 1;
    }
    
    // 设置运动学求解器
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;
    std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver;
    
    setup_kinematics(chain, fk_solver, ik_vel_solver, ik_pos_solver);
    
    // 正向运动学测试
    std::cout << std::endl;
    std::cout << "=== FK ===" << std::endl;
    
    std::vector<double> joint_positions = {0.2, 0.2, 0.3, 0.4, 0.5, 0.6};
    if (static_cast<int>(joint_positions.size()) != chain.getNrOfJoints()) {
        std::cout << "joints (" << joint_positions.size() << ") not match (" 
                  << chain.getNrOfJoints() << ")" << std::endl;
        joint_positions.assign(chain.getNrOfJoints(), 0.0);
    }
    
    KDL::Frame end_effector_frame;
    if (!perform_forward_kinematics(*fk_solver, joint_positions, end_effector_frame)) {
        std::cout << std::endl;
        std::cout << "Press Enter to exit..." << std::endl;
        std::cin.get();
        return 1;
    }
    
    // 逆运动学测试
    std::cout << std::endl;
    std::cout << "=== IK ===" << std::endl;
    
    std::vector<double> inverse_joints;
    if (perform_inverse_kinematics(*ik_pos_solver, chain, end_effector_frame, 
                                   joint_positions, inverse_joints)) {
        // 验证 IK 结果
        KDL::Frame verified_frame;
        if (perform_forward_kinematics(*fk_solver, inverse_joints, verified_frame)) {
            // 计算位置误差
            KDL::Vector verified_pos = verified_frame.p;
            KDL::Vector original_pos = end_effector_frame.p;
            
            double error_pos = sqrt(
                (verified_pos.x() - original_pos.x()) * (verified_pos.x() - original_pos.x()) +
                (verified_pos.y() - original_pos.y()) * (verified_pos.y() - original_pos.y()) +
                (verified_pos.z() - original_pos.z()) * (verified_pos.z() - original_pos.z())
            );
            std::cout << "error: " << std::fixed << std::setprecision(8) << error_pos << std::endl;
        }
    }
    
    std::cout << std::endl;
    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();
    
    return 0;
}

