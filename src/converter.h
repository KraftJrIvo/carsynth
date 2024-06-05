#include <Eigen/Core>
#include <Eigen/Geometry>

struct Pose
{
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();

    Eigen::Matrix4d matrix() const {
        Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
        res.block<3, 3>(0, 0) = rotation.matrix();
        res.block<3, 1>(0, 3) = position;
        return res;
    }

    static Pose fromMatrix(const Eigen::Matrix4d& mat) {
        Pose res;
        res.rotation = Eigen::Quaterniond(mat.block<3, 3>(0, 0));
        res.position = mat.block<3, 1>(0, 3);
        return res;
    }
    Pose()
    {
        position = Eigen::Vector3d::Zero();
        rotation = Eigen::Quaterniond::Identity();
    }
    Pose(const Eigen::Matrix4d& mat)
    {
        Pose pose = fromMatrix(mat);
        position = pose.position;
        rotation = pose.rotation;
    }
    Pose& operator=(const Eigen::Matrix4d& mat)
    {
        Pose pose = fromMatrix(mat);
        *this = pose;
        return *this;
    }
    Pose operator*(const Pose& p)
    {
        return fromMatrix(this->matrix() * p.matrix());
    }
    Pose inverse()
    {
        return Pose::fromMatrix(this->matrix().inverse());
    }
};

class GNSSconverter {
public:
	GNSSconverter() { _ned2enu << 0, 1, 0, 1, 0, 0, 0, 0, -1; }
	void setOrigin(const Eigen::Vector3d& pos);
	Eigen::Vector3d globe2enu(const Eigen::Vector3d& pos);
	Eigen::Vector3d globe2ecef(const Eigen::Vector3d& pos);
	Eigen::Vector3d enu2globe(const Eigen::Vector3d& enu);
	Eigen::Vector3d ecef2globe(const Eigen::Vector3d& xyz);
	Eigen::Vector3d ecef2enu(const Eigen::Vector3d& xyz);
	Pose ecef2enu(const Eigen::Vector3d& ecef, const Eigen::Quaterniond& ecef_q = Eigen::Quaterniond::Identity());
	Eigen::Quaterniond ecefRot2enuRot(const Eigen::Vector3d& posECEF, const Eigen::Vector3d& posENU, const Eigen::Quaterniond& q);
	Eigen::Matrix3d globe2enuJacobian(const Eigen::Vector3d& pos);
	bool originSet() { return _originSet; }
	Eigen::Matrix3d convertLocalNedGNSSCov2EnuOrigin(Eigen::Vector3d lla, Eigen::Matrix3d cov_ned);
private:
	Eigen::Matrix3d getEcef2EnuRot(const Eigen::Vector3d& lla);
	Eigen::Vector3d _origin;
	Eigen::Vector3d _originECEF;
	Eigen::Vector3d _xyz;
	const double _ecvR = 6378137;
	const double _b = 6356752.314245;
	const double _fEccen = 6.69437999014 * 0.001;
	const double _sEccen = 6.73949674228 * 0.001;
	const double _f_inv = 298.257224;
	const double _f = 1.0 / _f_inv;
	const double _e2 = 1 - (1 - _f) * (1 - _f);
	double _sinLatRef, _cosLatRef, _sinLonRef, _cosLonRef, _cRef;
	bool _originSet = false;
	Eigen::Matrix3d _ned2enu;
};