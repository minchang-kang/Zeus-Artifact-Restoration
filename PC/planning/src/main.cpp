#include <iostream>
#include <vector>
#include <stdexcept>
#include <cstring>

#include <boost/program_options.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/gp3.h>

// Noether
#include <noether_tpp/core/types.h>
#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <noether_tpp/mesh_modifiers/clean_data_modifier.h>
#include <noether_tpp/mesh_modifiers/normal_estimation_pcl.h>
#include <noether_tpp/mesh_modifiers/compound_modifier.h>
#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators/principal_axis_direction_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/aabb_center_origin_generator.h>
#include <noether_tpp/tool_path_modifiers/snake_organization_modifier.h>

// Unix socket
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace po = boost::program_options;

// ── 법선 추정 ─────────────────────────────────────────────────────────────────

pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormals(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius)
{
    auto tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);

    auto normals = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();
    ne.compute(*normals);

    auto cloud_with_normals = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    return cloud_with_normals;
}

// ── RANSAC 세그멘테이션: 평면(절단면) 인라이어만 추출 ─────────────────────────

pcl::PointCloud<pcl::PointNormal>::Ptr segmentBreakFace(
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals,
    double dist_threshold = 0.005)
{
    auto xyz = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::copyPointCloud(*cloud_with_normals, *xyz);

    // SACMODEL_PLANE (위치 기반만 사용) — 법선 추정이 불안정한 희소 데이터에 robust
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(2000);
    seg.setDistanceThreshold(dist_threshold);

    auto result = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    auto remaining_n = cloud_with_normals;
    auto remaining_xyz = xyz;

    // 두 절단면을 순서대로 추출 (RANSAC 2회)
    for (int pass = 0; pass < 2; ++pass)
    {
        if (remaining_xyz->size() < 10)
            break;

        seg.setInputCloud(remaining_xyz);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
        seg.segment(*inliers, *coeffs);

        if (inliers->indices.empty() || inliers->indices.size() < 5)
            break;

        std::cout << "[seg] pass " << pass + 1
                  << ": " << inliers->indices.size() << " inliers\n";

        pcl::ExtractIndices<pcl::PointNormal> extract_n;
        extract_n.setInputCloud(remaining_n);
        extract_n.setIndices(inliers);

        auto face = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        extract_n.setNegative(false);
        extract_n.filter(*face);
        *result += *face;

        auto rest_n = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        extract_n.setNegative(true);
        extract_n.filter(*rest_n);
        remaining_n = rest_n;

        pcl::ExtractIndices<pcl::PointXYZ> extract_xyz;
        extract_xyz.setInputCloud(remaining_xyz);
        extract_xyz.setIndices(inliers);
        auto rest_xyz = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        extract_xyz.setNegative(true);
        extract_xyz.filter(*rest_xyz);
        remaining_xyz = rest_xyz;
    }

    if (result->empty())
        throw std::runtime_error("RANSAC segmentation found no planar inliers");

    std::cout << "[seg] total break-face points: " << result->size() << "\n";
    return result;
}

// ── GreedyProjectionTriangulation: PointNormal → PolygonMesh ─────────────────

pcl::PolygonMesh buildMesh(
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, double radius)
{
    auto tree2 = pcl::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
    tree2->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gpt;
    gpt.setSearchRadius(radius * 3.0);
    gpt.setMu(2.5);
    gpt.setMaximumNearestNeighbors(100);
    gpt.setMaximumSurfaceAngle(M_PI / 4.0);
    gpt.setMinimumAngle(M_PI / 18.0);
    gpt.setMaximumAngle(2.0 * M_PI / 3.0);
    gpt.setNormalConsistency(false);
    gpt.setInputCloud(cloud_with_normals);
    gpt.setSearchMethod(tree2);

    pcl::PolygonMesh mesh;
    gpt.reconstruct(mesh);

    if (mesh.polygons.empty())
        throw std::runtime_error("Mesh reconstruction produced no triangles");

    std::cout << "[mesh] triangles: " << mesh.polygons.size() << "\n";
    return mesh;
}

// ── Noether 파이프라인 ────────────────────────────────────────────────────────

std::vector<noether::ToolPaths> planToolPaths(
    const pcl::PolygonMesh& mesh, double radius, double spacing)
{
    using namespace noether;

    // MeshModifier 체인 (ConstPtr = unique_ptr<const MeshModifier>)
    std::vector<MeshModifier::ConstPtr> mesh_mods;
    mesh_mods.push_back(std::make_unique<CleanData>());
    mesh_mods.push_back(std::make_unique<NormalEstimationPCLMeshModifier>(radius));
    MeshModifier::ConstPtr compound_mod =
        std::make_unique<CompoundMeshModifier>(std::move(mesh_mods));

    // Planner (먼저 일반 포인터로 생성 → 메서드 호출 → 이동)
    auto planner_raw = new PlaneSlicerRasterPlanner(
        std::make_unique<PrincipalAxisDirectionGenerator>(),
        std::make_unique<AABBCenterOriginGenerator>());
    planner_raw->setLineSpacing(spacing);
    planner_raw->generateRastersBidirectionally(true);
    ToolPathPlanner::ConstPtr planner(planner_raw);

    // ToolPathModifier
    ToolPathModifier::ConstPtr snake_mod = std::make_unique<SnakeOrganizationModifier>();

    ToolPathPlannerPipeline pipeline(
        std::move(compound_mod), std::move(planner), std::move(snake_mod));
    auto result = pipeline.plan(mesh);

    std::size_t total = 0;
    for (auto& tps : result)
        for (auto& tp : tps)
            for (auto& seg : tp)
                total += seg.size();

    std::cout << "[noether] waypoints: " << total << "\n";
    return result;
}

// ── Waypoint 평탄화 및 직렬화 ─────────────────────────────────────────────────

std::vector<double> flattenWaypoints(const std::vector<noether::ToolPaths>& all)
{
    std::vector<double> buf;
    for (auto& tool_paths : all)
        for (auto& tool_path : tool_paths)
            for (auto& segment : tool_path)
                for (auto& wp : segment)
                {
                    Eigen::Matrix4d m = wp.matrix();
                    for (int r = 0; r < 4; ++r)
                        for (int c = 0; c < 4; ++c)
                            buf.push_back(m(r, c));
                }
    return buf;
}

// ── Unix Domain Socket 전송 ───────────────────────────────────────────────────

void sendWaypoints(const std::string& socket_path, const std::vector<double>& flat)
{
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0)
        throw std::runtime_error("socket() failed: " + std::string(strerror(errno)));

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path) - 1);

    if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        close(fd);
        throw std::runtime_error("connect() failed: " + std::string(strerror(errno)));
    }

    uint32_t n = static_cast<uint32_t>(flat.size() / 16);
    if (send(fd, &n, sizeof(n), 0) < 0)
    {
        close(fd);
        throw std::runtime_error("send header failed");
    }
    if (!flat.empty())
    {
        std::size_t bytes = flat.size() * sizeof(double);
        if (send(fd, flat.data(), bytes, 0) < 0)
        {
            close(fd);
            throw std::runtime_error("send data failed");
        }
    }

    close(fd);
    std::cout << "[socket] sent " << n << " waypoints to " << socket_path << "\n";
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    std::string input_path;
    double radius = 0.01;
    double spacing = 0.005;
    std::string socket_path = "/tmp/planning.sock";
    std::string segment_mode = "none";

    po::options_description desc("Zeus Planning Process");
    desc.add_options()
        ("help,h",    "show help")
        ("input,i",   po::value<std::string>(&input_path)->required(), "PCD file path")
        ("radius,r",  po::value<double>(&radius)->default_value(0.01),  "normal estimation radius [m]")
        ("spacing,s", po::value<double>(&spacing)->default_value(0.005),"raster line spacing [m]")
        ("socket",    po::value<std::string>(&socket_path)->default_value("/tmp/planning.sock"), "unix socket path")
        ("segment",   po::value<std::string>(&segment_mode)->default_value("none"), "segmentation mode: none|ransac");

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")) { std::cout << desc; return 0; }
        po::notify(vm);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Argument error: " << e.what() << "\n" << desc;
        return 1;
    }

    try
    {
        // [2] PCD 로드
        auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile(input_path, *cloud) < 0)
            throw std::runtime_error("Failed to load PCD: " + input_path);
        std::cout << "[pcd] loaded " << cloud->size() << " points from " << input_path << "\n";

        // [3] 법선 추정
        auto cloud_n = estimateNormals(cloud, radius);

        // [4] 절단면 세그멘테이션
        if (segment_mode == "ransac")
            cloud_n = segmentBreakFace(cloud_n);
        else if (segment_mode != "none")
            throw std::runtime_error("Unknown --segment mode: " + segment_mode);

        // [5] Mesh 변환
        auto mesh = buildMesh(cloud_n, radius);

        // [6–9] Noether 파이프라인
        auto tool_paths = planToolPaths(mesh, radius, spacing);

        // [10] 직렬화 및 Socket 전송
        auto flat = flattenWaypoints(tool_paths);
        sendWaypoints(socket_path, flat);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[error] " << e.what() << "\n";
        return 1;
    }

    return 0;
}
