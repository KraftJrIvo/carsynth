#include "converter.h"

#define M_PI 3.1415926535897

void GNSSconverter::setOrigin(const Eigen::Vector3d& pos) {
	_origin = pos;
	_originECEF = globe2ecef(pos);
	_cosLatRef = cos(_origin.x() * M_PI / 180.);
	_sinLatRef = sin(_origin.x() * M_PI / 180.);
	_cosLonRef = cos(_origin.y() * M_PI / 180.);
	_sinLonRef = sin(_origin.y() * M_PI / 180.);
	_cRef = 1 / (sqrt(_cosLatRef * _cosLatRef + (1 - _f) * (1 - _f) * _sinLatRef * _sinLatRef));
	_xyz.x() = (_ecvR * _cRef + _origin.z()) * _cosLatRef * _cosLonRef;
	_xyz.y() = (_ecvR * _cRef + _origin.z()) * _cosLatRef * _sinLonRef;
	_xyz.z() = (_ecvR * _cRef * (1 - _e2) + _origin.z()) * _sinLatRef;
	_originSet = true;
}

Eigen::Vector3d GNSSconverter::globe2enu(const Eigen::Vector3d& pos) {
	double cosLat, sinLat, cosLong, sinLong, c, s, x, y, z;
	cosLat = cos(pos.x() * M_PI / 180.);
	sinLat = sin(pos.x() * M_PI / 180.);
	cosLong = cos(pos.y() * M_PI / 180.);
	sinLong = sin(pos.y() * M_PI / 180.);
	c = 1 / (sqrt(cosLat * cosLat + (1 - _f) * (1 - _f) * sinLat * sinLat));
	s = (1 - _f) * (1 - _f) * c;
	x = (_ecvR * c + pos.z()) * cosLat * cosLong;
	y = (_ecvR * c + pos.z()) * cosLat * sinLong;
	z = (_ecvR * s + pos.z()) * sinLat;
	Eigen::Vector3d enu;
	enu[0] = (-(x - _xyz.x()) * _sinLonRef) + ((y - _xyz.y()) * _cosLonRef);
	enu[1] = (-_cosLonRef * _sinLatRef * (x - _xyz.x())) - (_sinLatRef * _sinLonRef * (y - _xyz.y())) + (_cosLatRef * (z - _xyz.z()));
	enu[2] = (_cosLatRef * _cosLonRef * (x - _xyz.x())) + (_cosLatRef * _sinLonRef * (y - _xyz.y())) + (_sinLatRef * (z - _xyz.z()));
	return enu;
}

Eigen::Vector3d GNSSconverter::enu2globe(const Eigen::Vector3d& enu) {
	double eps, xd, yd, zd, xn, yn, zn, p, q, v, sinq, cosq, phi, lambda;
	eps = _e2 / (1 - _e2);
	xd = -_sinLonRef * enu[0] - _cosLonRef * _sinLatRef * enu[1] + _cosLatRef * _cosLonRef * enu[2];
	yd = _cosLonRef * enu[0] - _sinLatRef * _sinLonRef * enu[1] + _cosLatRef * _sinLonRef * enu[2];
	zd = _cosLatRef * enu[1] + _sinLatRef * enu[2];
	xn = xd + _xyz.x();
	yn = yd + _xyz.y();
	zn = zd + _xyz.z();
	p = sqrt(xn * xn + yn * yn);
	q = atan2(zn * _ecvR, p * _b);
	sinq = sin(q);
	cosq = cos(q);
	phi = atan2(zn + eps * _b * sinq * sinq * sinq, p - _e2 * _ecvR * cosq * cosq * cosq);
	lambda = atan2(yn, xn);
	v = _ecvR / (sqrt(1 - _e2 * sin(phi) * sin(phi)));
	Eigen::Vector3d pos;
	pos[0] = phi * 180. / M_PI;
	pos[1] = lambda * 180. / M_PI;
	pos[2] = p / cos(phi) - v;
	return pos;
}

Eigen::Vector3d GNSSconverter::ecef2globe(const Eigen::Vector3d& xyz) {
	Eigen::Vector3d llaWGS;
	double r = sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
	double Esq = _ecvR * _ecvR - _b * _b;
	double F_ = 54. * _b * _b * xyz.z() * xyz.z();
	double G = r * r + (1 - _fEccen) * xyz.z() * xyz.z() - _fEccen * Esq;
	double C = (_fEccen * _fEccen * F_ * r * r) / pow(G, 3);
	double S = cbrt(1 + C + sqrt(C * C + 2 * C));
	double P = F_ / (3 * pow((S + 1 / S + 1), 2) * G * G);
	double Q = sqrt(1 + 2 * _fEccen * _fEccen * P);
	double r_0 = -(P * _fEccen * r) / (1 + Q) + sqrt(0.5 * _ecvR * _ecvR * (1 + 1.0 / Q) - P * (1 - _fEccen) * xyz.z() * xyz.z() / (Q * (1 + Q)) - 0.5 * P * r * r);
	double U = sqrt(pow((r - _fEccen * r_0), 2) + xyz.z() * xyz.z());
	double V = sqrt(pow((r - _fEccen * r_0), 2) + (1 - _fEccen) * xyz.z() * xyz.z());
	double Z_0 = _b * _b * xyz.z() / (_ecvR * V);
	llaWGS[0] = 180. / M_PI * (atan((xyz.z() + _sEccen * Z_0) / r));
	llaWGS[1] = 180. / M_PI * (atan2(xyz.y(), xyz.x()));
	llaWGS[2] = U * (1 - _b * _b / (_ecvR * V));
	return llaWGS;
}

Eigen::Vector3d GNSSconverter::ecef2enu(const Eigen::Vector3d& xyz) {
	return globe2enu(ecef2globe(xyz));
}

Eigen::Quaterniond GNSSconverter::ecefRot2enuRot(const Eigen::Vector3d& posECEF, const Eigen::Vector3d& posENU, const Eigen::Quaterniond& q) {
	auto m = q.matrix();
	Eigen::Vector3d vx = posECEF + m.col(0);
	Eigen::Vector3d vy = posECEF + m.col(1);
	Eigen::Vector3d vz = posECEF + m.col(2);
	vx = (globe2enu(ecef2globe(vx)) - posENU).normalized();
	vy = (globe2enu(ecef2globe(vy)) - posENU).normalized();
	vz = (globe2enu(ecef2globe(vz)) - posENU).normalized();
	m.block<3, 1>(0, 0) = vx;
	m.block<3, 1>(0, 1) = vy;
	m.block<3, 1>(0, 2) = vz;
	return Eigen::Quaterniond(m);
}

Eigen::Vector3d GNSSconverter::globe2ecef(const Eigen::Vector3d& pos)
{
	const double f = (_ecvR - _b) / _ecvR;
	const double f_inv = 1.0 / f;

	const double a_sq = _ecvR * _ecvR;
	const double b_sq = _b * _b;
	const double e_sq = f * (2 - f);

	double cosLat, sinLat, cosLong, sinLong, N;
	cosLat = cos(pos.x() * M_PI / 180.);
	sinLat = sin(pos.x() * M_PI / 180.);
	cosLong = cos(pos.y() * M_PI / 180.);
	sinLong = sin(pos.y() * M_PI / 180.);
	N = _ecvR / sqrt(1 - e_sq * sinLat * sinLat);

	Eigen::Vector3d ecef;
	ecef[0] = (pos[2] + N) * cosLat * cosLong;
	ecef[1] = (pos[2] + N) * cosLat * sinLong;
	ecef[2] = (pos[2] + (1 - e_sq) * N) * sinLat;

	return ecef;
}

Pose GNSSconverter::ecef2enu(const Eigen::Vector3d& ecef, const Eigen::Quaterniond& ecef_q)
{
	Eigen::Vector3d ecefd = ecef - _originECEF;
	Eigen::Matrix3d ecef2enuRot = getEcef2EnuRot(_origin);

	Pose enu_pose;
	enu_pose.position = ecef2enuRot * ecefd;
	enu_pose.rotation = ecef2enuRot * ecef_q;

	return enu_pose;
}

Eigen::Matrix3d GNSSconverter::getEcef2EnuRot(const Eigen::Vector3d& lla)
{
	double cosLat, sinLat, cosLong, sinLong, N;
	cosLat = cos(lla.x() * M_PI / 180.);
	sinLat = sin(lla.x() * M_PI / 180.);
	cosLong = cos(lla.y() * M_PI / 180.);
	sinLong = sin(lla.y() * M_PI / 180.);

	Eigen::Matrix3d ecef2enuRot;
	ecef2enuRot(0, 0) = -sinLong;
	ecef2enuRot(0, 1) = cosLong;
	ecef2enuRot(0, 2) = 0;
	ecef2enuRot(1, 0) = -cosLong * sinLat;
	ecef2enuRot(1, 1) = -sinLat * sinLong;
	ecef2enuRot(1, 2) = cosLat;
	ecef2enuRot(2, 0) = cosLat * cosLong;
	ecef2enuRot(2, 1) = cosLat * sinLong;
	ecef2enuRot(2, 2) = sinLat;

	return ecef2enuRot;
}

Eigen::Matrix3d GNSSconverter::globe2enuJacobian(const Eigen::Vector3d& pos) {
	Eigen::Vector3d x0 = globe2enu(pos);
	double epsilon = 1.0;
	Eigen::Matrix3d jac;
	jac.block<3, 1>(0, 0) = (globe2enu({ pos.x() + epsilon, pos.y(), pos.z() }) - x0) / epsilon;
	jac.block<3, 1>(0, 1) = (globe2enu({ pos.x(), pos.y() + epsilon, pos.z() }) - x0) / epsilon;
	jac.block<3, 1>(0, 2) = (globe2enu({ pos.x(), pos.y(), pos.z() + epsilon }) - x0) / epsilon;
	return jac;
}

Eigen::Matrix3d GNSSconverter::convertLocalNedGNSSCov2EnuOrigin(Eigen::Vector3d lla, Eigen::Matrix3d cov_ned)
{
	Eigen::Matrix3d cov_enu_lla = _ned2enu * cov_ned * _ned2enu.transpose();
	Eigen::Matrix3d ecef2enu_lla = getEcef2EnuRot(lla);
	Eigen::Matrix3d ecef2enu_origin = getEcef2EnuRot(_origin);
	Eigen::Matrix3d enu_lla2enu_origin = ecef2enu_origin * ecef2enu_lla.inverse();
	Eigen::Matrix3d cov_enu_origin = enu_lla2enu_origin * cov_enu_lla * enu_lla2enu_origin.transpose();
	return cov_enu_origin;
}