---
layout: post
title: SLAM, Robotics 프로그래밍을 위한 3가지 코딩 꿀팁 (2)
subtitle: 2. Quaternion이나 Eular angles을 변수화할 때 순서를 명시하자
tags: [SLAM, Robotics, ROS]
comments: true
---

(Cont'd)

### 2. Quaternion이나 Eular angles을 변수화할 때 순서를 명시하자

만약 `vector<double> q`라는 크기 4의, quaternion을 나타내는 변수가 있다고 가정하자. 이를 오일러각 혹은 matrix의 형태로 변환하거나 상대적 회전을 계산할 때 quaternion의 요소 값을 불러와야 된다. 여기서 큰 문제가 있는데, 위의 변수 `q`가 **XYZW 순인지 WXYZ 순인지 알 수 없다는 것이다.** 

꿀팁 (2)에서 강조하고자하는 바는, Quaternion을 표현할 때 XYZW 순으로 나타내는 라이브러리도 있고, WXYZ 순으로 나타내기도 한다는 것이다. 특히나 로봇분야에서 많이 쓰이는 라이브러리인 Eigen과 ROS tf 내부의 quaternion을 다루는 순서가 아래와 같이 다르다: 

```cpp
// `ori`라는, x, y, z, and w를 멤버 변수로 가지는 quaternion struct가 있다고 가정
// Quaternion 값의 순서가 라이브러리마다 다르다!
// Eigen: w, x, y, z order
// https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html#a3eba7a582f77a8f30525614821d7056f
// tf   : x, y, z, w order
Eigen::Quaterniond q_e(ori.w, ori.x, ori.y, ori.z);
tf::Quaternion q_t(ori.x, ori.y, ori.z, ori.w);
```

이는 라이브러리의 메뉴얼을 꼼꼼히 읽어보지 않은 이를 엄청난 디버깅 지옥(!)에 빠트릴 수 있다. 더 혼란스러운 건 `q_e.coeffs()`로 quaternion의 element를 확인할 때는 [qx, qy, qz, qw](https://eigen.tuxfamily.org/dox/classEigen_1_1QuaternionBase.html#ae61294790c0cc308d3f69690a657672c) 순이다. 비단 Eigen과 ROS tf 뿐만 아니라, 다른 언어(e.g. python이나 C# 등)의 라이브러리에서도 똑같이 어떤 곳에서는 XYZW를 쓰고 어떤 곳에서는 WXYZ를 쓴다. 
실례로, 필자도 python에 있는 [pyquaternion](http://kieranwynn.github.io/pyquaternion/)을 사용하여 3D pose를 나타낼 때, XYZW 순이겠거니 지레짐작했다가 pose가 계속 이상하게 표현되어 한 이틀가량 모든 코드를 디버깅하느라 진을 뺀 적이 있다 (pyquaternion은 Eigen과 마찬가지로 WXYZ form을 따른다).

### 꿀팁 (2)의 결론 및 해결책

따라서, 이러한 문제를 사전에 방지하기 위해, quaternion이나 euler angles을 나타낼 때에는 코드 작성자가 생각한 순서를 간단히 주석을 통해 나타내주는 것이 좋다 물론 [클린 코드](https://github.com/scvgoe/clean-code-summary)에 따르면, 주석 없이도 변수의 의미를 표현할 수 있는 것이 바람직하지만, 그렇다고 일시적으로 사용할 변수를 `q_wxyz`라고 표현하는 게 좀 더 지저분해 보이는 것 같다 (지극히 개인적 의견).

해당 꿀팁의 좋은 예시로는, FAST-LIO의 `/include/use-ikform.cpp` 내부의 `SO3ToEuler` 함수가 있다:
 
```cpp
vect3 SO3ToEuler(const SO3 &orient) 
{
	Eigen::Matrix<double, 3, 1> _ang;
	Eigen::Vector4d q_data = orient.coeffs().transpose();
	// HT: quaternion의 요소의 순서를 주석으로 명시함
	//scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2];
	double sqw = q_data[3]*q_data[3];
	double sqx = q_data[0]*q_data[0];
	double sqy = q_data[1]*q_data[1];
	double sqz = q_data[2]*q_data[2];
	double unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor
	double test = q_data[3]*q_data[1] - q_data[2]*q_data[0];

	if (test > 0.49999*unit) { // singularity at north pole
	
		_ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
	if (test < -0.49999*unit) { // singularity at south pole
		_ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
		
	_ang <<
			std::atan2(2*q_data[0]*q_data[3]+2*q_data[1]*q_data[2] , -sqx - sqy + sqz + sqw),
			std::asin (2*test/unit),
			std::atan2(2*q_data[2]*q_data[3]+2*q_data[1]*q_data[0] , sqx - sqy - sqz + sqw);
	double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
	vect3 euler_ang(temp, 3);
	// HT: Roll-pitch-Yaw 순이라고 친절히 명시
	// euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
	return euler_ang;
}
```

위의 함수에서는 각각 quaternion과 euler angle을 다룰 때 값의 순서에 대해서 
```cpp
//scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2]; 
```
```cpp
// euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
```
와 같이 명시해둔 것이 눈에 띈다. 코드를 작성하고 있는 당사자 입장에서는 '당연한 걸 왜 나타내야 해?'라고 생각할지 모르지만, 그 코드를 읽는 다른 사람에게는 이 간단한 주석 한 줄이 많은 시간을 아끼게 해준다. 만약 위에 `euler_ang`라는 변수의 순서에 대한 얘기가 없으면, 함수의 input-output만 확인하고 싶을 때 `_ang`라는 변수에서 값을 할당할 때 사용하는 부분의 의미를 자세히 살펴봐야 한다. 
그 결과, 각각의 수식의 의미가 무엇인지(즉 quaternion으로 roll-pitch-yaw를 구하는 수학적 표현)를 추가적으로 찾아봐야 하기 때문에, 코드를 살펴보는데 번거로움을 야기한다. 

물론 [클린 코드](https://github.com/scvgoe/clean-code-summary)에 따라 `SO3toRPY`라고 함수명을 지어 근원적 문제를 해결하는 것도 좋은 방법이다. 좀더 input-ouput이 명확해지기 때문이다. 하지만 그렇지 못한 경우라면, 회전에 대해서 변수를 선언할 때는 a) Eigen이나 ROS tf같이 이미 struct로 제공하는 값들을 사용하거나 b) 주석으로 해당 리스트의 순서가 각각 어떤 값을 나타내는지 표헌해주는 것이 좋다.

---

P.S. SLAM의 코드를 작성하다보면 수식 전개를 하는 부분을 피할 수 없다. 이런 상황에 직면했을 때, 개인적인 경험으로는 수학수학한 부분은 자료의 주소를 아예 적어두는 것도 미래의 '나'에게 좋은 방법이라고 생각된다. 예시는 [Quatro](https://github.com/url-kaist/Quatro/blob/79faf8800805f53fd0c7e94315a966381ed1d5ed/include/teaser/utils.h) 코드 내에 있는 svd를 구하는 부분. 원래 TEASER++ 내부에 있는 코드인데, 아래 수식을 잘 이해하고 싶은 이에게는 http://igl.ethz.ch/projects/ARAP/svd_rot.pdf를 참고하라고 주석으로 되어 있다. 이를 통해 두 가지 장점을 얻을 수 있는데, 첫 째로는 수식의 정확한 이해를 알고 싶은 이에게는 코드의 의미를 더 잘 이해할 수 있게 가이드를 주고, 두번 째로는 pdf 상의 수학에서 사용한 변수와 동일한 변수를 코드 상에서 사용하여 코드의 간결성을 잘 보존할 수 있게 해다.

```cpp준
/**
 * Helper function to use svd to estimate rotation.
 * Method described here: http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
 * @param X
 * @param Y
 * @return a rotation matrix R
 */
inline Eigen::Matrix3d svdRot(const Eigen::Matrix<double, 3, Eigen::Dynamic>& X,
                              const Eigen::Matrix<double, 3, Eigen::Dynamic>& Y,
                              const Eigen::Matrix<double, 1, Eigen::Dynamic>& W,
                              int static_count) {
  // Assemble the correlation matrix H = X * Y'
  Eigen::Matrix3d H = X * W.asDiagonal() * Y.transpose();

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  if (U.determinant() * V.determinant() < 0) {
    V.col(2) *= -1;
  }

  return V * U.transpose();
}
```

---


SLAM, Robotics 프로그래밍을 위한 3가지 코딩 꿀팁 시리즈입니다.

1. [SLAM, Robotics 프로그래밍을 위한 3가지 코딩꿀팁 (1)](https://limhyungtae.github.io/2022-09-04-SLAM,-Robotics-%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%EC%9D%84-%EC%9C%84%ED%95%9C-3%EA%B0%80%EC%A7%80-%EC%BD%94%EB%94%A9-%EA%BF%80%ED%8C%81-(1)/)
2. [SLAM, Robotics 프로그래밍을 위한 3가지 코딩꿀팁 (2)](https://limhyungtae.github.io/2022-09-04-SLAM,-Robotics-%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%EC%9D%84-%EC%9C%84%ED%95%9C-3%EA%B0%80%EC%A7%80-%EC%BD%94%EB%94%A9-%EA%BF%80%ED%8C%81-(2)/)
3. [SLAM, Robotics 프로그래밍을 위한 3가지 코딩꿀팁 (3)](https://limhyungtae.github.io/2022-09-04-SLAM,-Robotics-%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%EC%9D%84-%EC%9C%84%ED%95%9C-3%EA%B0%80%EC%A7%80-%EC%BD%94%EB%94%A9-%EA%BF%80%ED%8C%81-(3)/)
