#include "libobsensor/ObSensor.hpp"
#include <cerrno>
#include <cmath>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <thread>

#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
int kbhit(void) {
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF) {
		ungetc(ch, stdin);
		return 1;
	}
	return 0;
}

int getch(void) {
	struct termios oldt, newt;
	int ch;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
}
#endif

#define KEY_ESC 27

namespace {

// Small filesystem helpers used by capture output path handling.

bool IsDirectory(const std::string& path) {
	struct stat st;
	if(stat(path.c_str(), &st) != 0) {
		return false;
	}
	return S_ISDIR(st.st_mode);
}

bool EnsureDirectory(const std::string& path) {
	if(IsDirectory(path)) {
		return true;
	}
	if(mkdir(path.c_str(), 0755) == 0) {
		return true;
	}
	return errno == EEXIST && IsDirectory(path);
}

std::string JoinPath(const std::string& a, const std::string& b) {
	if(a.empty()) {
		return b;
	}
	if(a.back() == '/') {
		return a + b;
	}
	return a + "/" + b;
}

}  // namespace

void saveRGBPointsToPly(
	const std::shared_ptr<ob::Frame>& frame,
	const std::string& file_name,
	double point_to_meter) {
	// Convert SDK frame buffer to ASCII PLY while skipping zero vectors.
	// The saved coordinates are converted to meters using point_to_meter.
	int points_size = frame->dataSize() / sizeof(OBColorPoint);
	FILE *fp = fopen(file_name.c_str(), "wb+");

	if(!fp) {
		throw std::runtime_error("Failed to open file for writing");
	}

	OBColorPoint *point = (OBColorPoint *)frame->data();
	int valid_points_count = 0;
	static const auto min_distance = 1e-6;

	for(int i = 0; i < points_size; i++) {
		if(fabs(point->x) >= min_distance || fabs(point->y) >= min_distance || fabs(point->z) >= min_distance) {
			valid_points_count++;
		}
		point++;
	}

	point = (OBColorPoint *)frame->data();

	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "element vertex %d\n", valid_points_count);
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar red\n");
	fprintf(fp, "property uchar green\n");
	fprintf(fp, "property uchar blue\n");
	fprintf(fp, "end_header\n");

	for(int i = 0; i < points_size; i++) {
		if(fabs(point->x) >= min_distance || fabs(point->y) >= min_distance || fabs(point->z) >= min_distance) {
			fprintf(fp, "%.3f %.3f %.3f %d %d %d\n",
					point->x * point_to_meter,
					point->y * point_to_meter,
					point->z * point_to_meter,
					(int)point->r, (int)point->g, (int)point->b);
		}
		point++;
	}

	fflush(fp);
	fclose(fp);
}

int main(int argc, char **argv) try {
	// Capture entry:
	//   manual: save only on key press
	//   timer : save periodically at fixed interval
	ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);
	const double point_to_meter = 0.001;
	const std::string default_raw_data_dir = "/home/pi/repo/orbbec_reconstruction/data/raw_data";

	bool timer_mode = false;
	int auto_save_interval_ms = 0;
	std::string raw_data_dir = default_raw_data_dir;
	if(argc >= 2 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
		std::cout << "Usage:\n"
				  << "  " << argv[0] << " manual [raw_data_dir]\n"
				  << "  " << argv[0] << " timer <interval_ms> [raw_data_dir]\n\n"
				  << "Arguments:\n"
				  << "  manual                  Save only when pressing R/r\n"
				  << "  timer                   Save automatically every interval_ms\n"
				  << "  interval_ms             Auto-save interval in milliseconds, must be > 0\n"
				  << "  raw_data_dir            Output directory for captured clouds\n"
				  << "                          Default: " << default_raw_data_dir << std::endl;
		return 0;
	}
	if(argc < 2) {
		std::cerr << "Usage:\n"
				  << "  " << argv[0] << " manual [raw_data_dir]\n"
				  << "  " << argv[0] << " timer <interval_ms> [raw_data_dir]\n"
				  << "Try --help for full argument descriptions." << std::endl;
		return 1;
	}

	std::string save_mode = argv[1];
	if(save_mode == "manual") {
		timer_mode = false;
	}
	else if(save_mode == "timer") {
		if(argc < 3) {
			std::cerr << "timer mode requires interval_ms" << std::endl;
			return 1;
		}
		auto_save_interval_ms = std::stoi(argv[2]);
		if(auto_save_interval_ms <= 0) {
			std::cerr << "interval_ms must be > 0" << std::endl;
			return 1;
		}
		timer_mode = true;
		if(argc >= 4) {
			raw_data_dir = argv[3];
		}
	}
	else {
		std::cerr << "Unknown mode: " << save_mode << ". Use manual or timer." << std::endl;
		return 1;
	}

	if(save_mode == "manual" && argc >= 3) {
		raw_data_dir = argv[2];
	}

	if(!EnsureDirectory(raw_data_dir)) {
		std::cerr << "Failed to create/access raw_data_dir: " << raw_data_dir
				  << " (" << std::strerror(errno) << ")" << std::endl;
		return 1;
	}
	std::cout << "raw_data_dir=" << raw_data_dir << std::endl;

	ob::Pipeline pipeline;
	std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

	// Enable color stream first, because depth profile is selected to align with color.
	std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
	try {
		auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
		if(colorProfiles) {
			auto profile = colorProfiles->getProfile(OB_PROFILE_DEFAULT);
			colorProfile = profile->as<ob::VideoStreamProfile>();
		}
		config->enableStream(colorProfile);
	}
	catch(ob::Error &e) {
		std::cerr << "Error: this device does not support color sensor." << std::endl;
		return -1;
	}

	std::shared_ptr<ob::StreamProfileList> depthProfileList;
	OBAlignMode alignMode = ALIGN_D2C_HW_MODE;
	// Try hardware depth-to-color alignment first, fallback to software alignment.
	try {
		depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
		if(depthProfileList->count() == 0) {
			depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
			if(depthProfileList->count() > 0) {
				alignMode = ALIGN_D2C_SW_MODE;
			}
		}

		if(depthProfileList->count() > 0) {
			std::shared_ptr<ob::StreamProfile> depthProfile;
			try {
				depthProfile = depthProfileList->getVideoStreamProfile(
					OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, colorProfile->fps());
			}
			catch(...) {
				depthProfile = nullptr;
			}

			if(!depthProfile) {
				depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT);
			}
			config->enableStream(depthProfile);

			try {
				pipeline.enableFrameSync();
			}
			catch(ob::Error &e) {
				std::cerr << "Warning: frame sync is not supported by this device." << std::endl;
			}
		}
	}
	catch(ob::Error &e) {
		std::cerr << "Error: failed to get D2C depth profile list." << std::endl;
		return -1;
	}

	config->setAlignMode(alignMode);
	pipeline.start(config);

	ob::PointCloudFilter pointCloud;
	auto cameraParam = pipeline.getCameraParam();
	pointCloud.setCameraParam(cameraParam);

	if(timer_mode) {
		std::cout << "Current mode: timer capture" << std::endl;
		std::cout << "Auto-save one full point cloud every " << auto_save_interval_ms << " ms." << std::endl;
	}
	else {
		std::cout << "Current mode: manual capture" << std::endl;
		std::cout << "Press R/r to save one full point cloud to a PLY file." << std::endl;
	}
	std::cout << "Press ESC to exit." << std::endl;

	int count = 0;
	int num = 0;
	// Shared save routine used by both timer and manual triggers.
	auto save_cloud_from_frameset = [&](const auto &frameset, const std::string &trigger) -> bool {
		try {
			if(frameset == nullptr || frameset->depthFrame() == nullptr || frameset->colorFrame() == nullptr) {
				std::cout << "[" << trigger << "] invalid frame set" << std::endl;
				return false;
			}

			auto depthValueScale = frameset->depthFrame()->getValueScale();
			pointCloud.setPositionDataScaled(depthValueScale);
			pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
			std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
			if(!frame) {
				std::cout << "[" << trigger << "] failed to generate point cloud" << std::endl;
				return false;
			}

			// Output naming convention required by downstream tools.
			std::string filename = JoinPath(raw_data_dir, "data_" + std::to_string(num) + ".ply");
			num++;
			saveRGBPointsToPly(frame, filename, point_to_meter);
			std::cout << "[" << trigger << "] point cloud saved to: " << filename << std::endl;
			return true;
		}
		catch(std::exception &e) {
			std::cout << "[" << trigger << "] failed to save point cloud: " << e.what() << std::endl;
			return false;
		}
	};

	auto last_auto_save_time = std::chrono::steady_clock::now();
	while(true) {
		// Poll frames continuously. Timer/manual logic decides when to save.
		auto frameset = pipeline.waitForFrames(100);

		if(timer_mode) {
			auto now = std::chrono::steady_clock::now();
			auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_auto_save_time).count();
			if(elapsed_ms >= auto_save_interval_ms) {
				save_cloud_from_frameset(frameset, "timer");
				last_auto_save_time = now;
			}
		}

		if(kbhit()) {
			int key = getch();
			if(key == KEY_ESC) {
				break;
			}

			if(!timer_mode && (key == 'R' || key == 'r')) {
				// Retry several times to avoid saving invalid transient frames.
				count = 0;
				while(count++ < 10) {
					auto retry_frameset = pipeline.waitForFrames(100);
					if(retry_frameset != nullptr && retry_frameset->depthFrame() != nullptr && retry_frameset->colorFrame() != nullptr) {
						save_cloud_from_frameset(retry_frameset, "manual");
						break;
					}
					else {
						std::cout << "failed to fetch frame set, retrying..." << std::endl;
						std::this_thread::sleep_for(std::chrono::milliseconds(50));
					}
				}
			}
		}
	}

	pipeline.stop();
	return 0;
}
catch(ob::Error &e) {
	std::cerr << "Orbbec SDK error:" << std::endl;
	std::cerr << "  Function: " << e.getName() << std::endl;
	std::cerr << "  Arguments: " << e.getArgs() << std::endl;
	std::cerr << "  Message: " << e.getMessage() << std::endl;
	std::cerr << "  Type: " << e.getExceptionType() << std::endl;
	exit(EXIT_FAILURE);
}
catch(std::exception &e) {
	std::cerr << "Standard exception: " << e.what() << std::endl;
	exit(EXIT_FAILURE);
}

