---
layout: post
title: Tailscale로 학교 데탑에 어디서든 SSH 접속하기
subtitle: 방화벽도 NAT도 신경 안 쓰고 내 컴퓨터에 들어가는 법
tags: [Tailscale, SSH, Productivity, Networking]
comments: true
description: 학교 NAT/방화벽 뒤에 있는 연구실 데스크탑에 외부에서 SSH로 접근하는 법을 Tailscale 기반 mesh VPN으로 정리한다. 설치부터 MagicDNS, Tailscale SSH까지.
permalink: /2026/05/22/tailscale-ssh-into-school-desktop/
redirect_from:
  - '/2026-05-22-Tailscale로 학교 데탑에 어디서든 SSH 접속하기/'
  - '/2026-05-22-Tailscale로-학교-데탑에-어디서든-SSH-접속하기/'
---

## 들어가며

대학원생이라면 한 번쯤 이런 상황을 겪는다.

학교 연구실 데스크탑에는 GPU도 박혀 있고 코드도 다 세팅돼 있다.
그런데 주말에 집에서 잠깐 실험 돌려보고 싶다.
또는 출장 가서 호텔 wifi로 잠깐 결과 확인하고 싶다.
**그런데 학교 네트워크는 외부에서 SSH가 안 뚫린다.**

NAT 뒤에 있고, 방화벽도 있고, 학교 IT팀에 포트 열어달라고 부탁하는 것도 보통 일이 아니다.
VPN을 깔자니 학교 VPN은 또 매번 끊기고 느리다.

그러다 [Tailscale](https://tailscale.com/)을 알게 됐는데, 솔직히 처음 깔아보고 좀 놀랐다.
**학교 데탑이랑 내 맥북을 같은 LAN에 있는 것처럼 묶어버린다.**
방화벽 설정도, 포트 포워딩도, 공인 IP도 필요 없다. 양쪽에 깔고 같은 계정으로 로그인하면 끝이다.

각설하고, 출발해보자.

---

## Tailscale이 뭘 해주는가

한 줄로 정리하면, **WireGuard 기반의 mesh VPN**이다.

뜻을 풀면 이렇다.
- 내가 가진 모든 디바이스(맥북, 학교 데탑, 집 서버 등)에 Tailscale을 깔고 같은 계정으로 로그인하면,
- 그 디바이스들끼리만 통하는 가상의 LAN(`100.x.x.x` 대역)이 생긴다.
- 디바이스 간 통신은 P2P로 직결되고, NAT를 못 뚫는 경우에만 Tailscale의 DERP relay 서버를 잠깐 거친다.

학교 데탑 입장에서는 *바깥에서 들어오는 연결*이 아니라 *자기가 Tailscale 서버에 나가서 연결을 유지하는* 모양새이다.
대부분의 방화벽은 outbound 연결은 허용하니까, **포트 하나도 새로 열 필요가 없다.**
신기한 건 이 지점이다.

---

## 설치

### 1. 학교 데탑 (Ubuntu 가정)

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

`sudo tailscale up`을 치면 터미널에 인증 URL이 뜬다.
브라우저로 접속해서 본인 계정(Google, GitHub 등 사용 가능)으로 로그인하면 그 머신이 본인의 tailnet에 등록된다.

```bash
tailscale ip -4
# → 100.x.x.x 형태의 주소가 뜸
```

이게 학교 데탑의 *Tailscale IP*이다. 이 IP는 본인 디바이스끼리만 보이고, 본인이 어디로 이동해도 바뀌지 않는다.

### 2. 내 노트북 (macOS 가정)

```bash
brew install --cask tailscale
```

또는 [공식 다운로드 페이지](https://tailscale.com/download)에서 앱을 받는다.
앱 실행 후 학교 데탑과 **같은 계정**으로 로그인하면 끝이다.

이 시점에서 노트북에서 `tailscale status`를 치면 학교 데탑이 본인의 tailnet 멤버로 보인다.

---

## SSH 접속

이제 카페든 호텔이든 어디서든, 노트북에서 그냥 이렇게 치면 된다.

```bash
ssh user@100.x.x.x
```

학교 네트워크에 있는 것과 완전히 동일하게 들어간다.
공인 IP도, 포트 포워딩도, 학교 VPN도 필요 없다.

### MagicDNS로 IP 외우지 않기

`100.x.x.x`를 매번 치기 귀찮으니, Tailscale admin 콘솔에서 **MagicDNS**를 켜자.
그러면 디바이스 이름(예: `lab-desktop`)으로 바로 접속할 수 있다.

```bash
ssh user@lab-desktop
```

`~/.ssh/config`에 굳이 등록하지 않아도 된다. tailnet 안에서는 디바이스 이름이 곧 hostname이다.

### Tailscale SSH (선택)

한 발 더 나아가, 학교 데탑에서 이렇게 켜두면

```bash
sudo tailscale up --ssh
```

**SSH key 관리조차 Tailscale이 대신해준다.**
즉, 노트북에 새 SSH key를 깔거나 `authorized_keys`에 등록하는 과정 없이, Tailscale 인증 자체를 SSH 인증으로 쓴다.
디바이스를 잃어버렸을 때 admin 콘솔에서 그 디바이스만 떼어내면 접근이 즉시 차단되니, key 관리 측면에서도 깔끔하다.

> 다만 본인은 OpenSSH key 기반 인증이 익숙해서 그냥 일반 `ssh`를 쓰고 있다. 본인 취향대로 고르면 된다.

---

## 어디까지 되나

SSH만 되는 게 아니다. tailnet 안의 디바이스끼리는 *모든 포트*가 그냥 열려 있는 것처럼 동작한다.

실제로 자주 쓰는 패턴 몇 가지.

- **Jupyter / TensorBoard 원격 접속**: 학교 데탑에서 `jupyter notebook --no-browser --port=8888`을 띄워두고, 노트북 브라우저로 `http://lab-desktop:8888` 접속.
- **VS Code Remote-SSH**: 그냥 `lab-desktop`을 host로 등록해두면 어디서든 학교 데탑의 코드를 IDE에서 직접 편집할 수 있다. 본인은 이 조합이 가장 효용이 크다.
- **파일 복사**: `scp some_result.bag user@lab-desktop:~/`처럼 그대로 동작한다.
- **집 NAS 마운트, 서버 간 데이터 동기화** 등도 같은 원리로 다 된다.

---

## 정리

- 학교 데탑에 외부에서 접근하고 싶은데 방화벽/NAT 때문에 막혀 있다면, **Tailscale을 깔자**. 포트 하나 열지 않고 해결된다.
- 양쪽 디바이스에 깔고 **같은 계정으로 로그인**하면 끝. `100.x.x.x` IP 또는 MagicDNS hostname으로 SSH를 친다.
- SSH key 관리도 귀찮으면 `tailscale up --ssh`로 Tailscale에 위임할 수 있다.
- 개인용은 무료 플랜으로 디바이스 100개까지 묶을 수 있다. 대학원생 입장에서는 사실상 무제한이다.

한 번 세팅해두면 *학교에 가야만 쓸 수 있는 컴퓨터*라는 개념 자체가 사라진다.
연구 효율 측면에서 체감 차이가 큰 도구이니, 아직 안 써봤다면 오늘 한 번 깔아보길 권한다.

---

## References

- [Tailscale 공식 문서](https://tailscale.com/kb/)
- [Tailscale SSH](https://tailscale.com/kb/1193/tailscale-ssh/)
- [MagicDNS](https://tailscale.com/kb/1081/magicdns/)
