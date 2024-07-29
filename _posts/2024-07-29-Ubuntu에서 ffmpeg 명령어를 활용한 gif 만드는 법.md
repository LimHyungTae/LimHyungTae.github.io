---
layout: post
title: Ubuntu에서 ffmpeg 명령어를 활용한 gif 만드는 법
subtitle: How to transform mp4 to gif
tags: [Ubuntu, ffmpeg, gif, mp4]
comments: true
---

이번 글에서는 `ffmpeg`를 통해 **효과적으로** gif를 만드는 방법에 대해 알아보자.

## Why GIF?

왜 비디오를 굳이 gif로 바꿔야 할까? 나는 주로 LinkedIn이나 Github README.md에 글을 올릴 때, gif를 활용하는데, 이는 내가 홍보하고 싶은 것을 좀 더 동적으로 보여주기 위함이다.
LinkedIn에 보면 논문 홍보를 위해 논문 첫 페이지 캡쳐 + 주저리주저리로 글을 써둔 이들이 많은데, 솔직히 말하면 눈에 잘 띄지 않는다. 그리고 기억에도 잘 안 남는다.
그러나 gif를 활용하면, 그래도 우연히 LinkedIn을 켠 사람에게 큰 임팩트를 줄 수 있는 것 같다(이 전략으로 현재 LinkedIn follower 5K명 달성,,,이 글을 읽는 이라면 gif로 글을 홍보하는 걸 늘 추천한다).

이러한 나의 경험은 예전에 네이버랩스에 있을 때 [김상배 교수님](https://meche.mit.edu/people/faculty/SANGBAE@MIT.EDU)께서 세미나에서도 뒷받침 되는데, 
김상배 교수님께서 다음과 같은 말씀을 해주신 적이 있으시다:

> 우리 인간의 눈은, event camera와 유사해서, 정적인 것보다는 무언가 움직이는 것에 더 집중을 잘할 가능성이 높다.
> 그도 그럴 것이, 약 10,000년 전만 하더라도 사냥을 위해 벌판을 뛰어다녔을 텐데, 이 때는 정적인 배경에 집중하는 것보다는 동적으로 움직이는 물체를 눈으로 좇는 것이 생존에 유리하기 때문이다. 

아무튼 홍보를 위해서라면 움직이는 무언가를 넣자! (그래서 필자는 학회에서 발표할 때도 정적으로 넣기 보다는 GIF로 움직이는 것들을 넣으려고 아둥바둥함)

## How to use

사실 요즘 시대에는 'ChatGPT한테 물어보면 다 되는 거 아님?'이라고 생각할 수 있지만, 분석해보고 싶은 것이 있어서 이렇게 글을 쓴다.

ChatGPT한테 물어보면 아래와 같이 가르쳐주는데,

```angular2html
ffmpeg -i input.mp4 -vf "fps=10,scale=1000:-1:flags=lanczos" output-long.gif
```

위에서 fps나 scale을 조절하면 gif의 크기나 속도를 조절할 수 있다(경험 상, 두 값이 커질수록 용량은 커진다. 위의 옵션은 Linkedin이나 README.md에 글을 게시할 때 적절한 옵션인 듯).

## Option

위에서 `lanczos`는 interpolation 방법을 나타낸다. 그럼 *어떤 option이 가장 효율적이고 퀄리티가 좋을까?* 찾아보니 후보군이 4개 있었다.

1. `bicubic`

   * 품질: 고품질. 부드러운 이미지와 선명한 엣지를 제공.
   * 성능: Bilinear보다 느림.
   * 특성: 주변 16개의 픽셀을 사용하여 보간, 더 정밀한 스케일링.

2. `sinc`

   * 품질: 매우 고품질. 선명한 엣지를 제공.
   * 성능: 느림.
   * 특성: Sinc 함수를 사용하여 보간.
   
3. `lanczos`

   * 품질: 매우 고품질. 선명하고 디테일한 이미지.
   * 성능: 느림.
   * 특성: Sinc 함수 기반의 창 함수를 사용하여 보간.

4. `spline`

   * 품질: 고품질. 매우 부드럽고 자연스러운 이미지.
   * 성능: 중간에서 느림.
   * 특성: 스플라인 보간법을 사용하여 부드럽고 자연스러운 결과 제공.

동일한 mp4로 네 개의 interpolation 방법을 적용해보니, 우선 생성된 gif의 용량의 크기는 `sinc (26 MB) > lanczos (23.3 MB) > spline (23.1 MB) > bicubic (22.5 MB)`이다.
근데 또 퀄리티를 비교해보면, gif를 확대해서 보았을 때 sinc로 gif를 생성했을 때가 lanczos보다 더 noise가 보이는 것을 육안으로 확인했다.  


### **sinc**로 생성한 gif 

![](/img/output-long-sinc.gif)

(확대해서 보면 origin 근처에 회색과 빨간색의 noise가 좀 더 생성되는 것을 볼 수 있음)
### **lanczos**로 생성한 gif 

![](/img/output-long-lanczos.gif)

## Conclusion

역시 ChatGPT 선생님이 옳았다. gif를 만들 때는 `lanczos`를 사용하는 것이 가장 효율적이고 퀄리티가 좋다!
(위의 gif는 [나의 이전 연구인 Patchwork](https://github.com/LimHyungTae/patchwork)를 visualization한 것임!)
