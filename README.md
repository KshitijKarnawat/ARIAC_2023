# ARIAC 2023 - Group 3

## Overview

This repository contains the code for the Course ENPM663 - Building a Manufacturing Robotic Software System
The course will focus on the development of a simulation-based control system that will address challenges presented in the Agile Robotics for Industrial Automation Competition(ARIAC)

## Team Members

- Sanchit Kedia (UID: 119159620)
- Adarsh Malapaka (UID: 118119625)
- Tanmay Haldankar (UID: 119175460)
- Sahruday Patti (UID: 118218366)
- Kshitij Karnawat (UID: 119188651)

## Dependencies

- [ROS2(Galactic)](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
- [ARIAC 2023 Workspace](https://github.com/usnistgov/ARIAC)
- [Ubuntu 20.04 LTS](https://releases.ubuntu.com/focal/)

## Build Package

```sh
source /opt/ros/galactic/setup.bash
source <Your workspace>/install/setup.bash
cd <Your ROS2 workspace src folder>
git clone https://github.com/Sanchitkedia/ARIAC_2023.git group3
cd ..
rosdep update --include-eol-distros
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select group3
```

## Run Package

```sh
source /opt/ros/galactic/setup.bash
source <Your workspace>/install/setup.bash
ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa2
ros2 run group3 group3
```

## Package Structure

```txt
.
├─ CMakeLists.txt
├─ LICENSE.md
├─ README.md
├─ document
│  └─ Activity_Diagram_v1.jpg
├─ group3
│  └─ __init__py
├─ include
│  └─ group3
│     ├─ ariac_competition.hpp
│     ├─ ceiling_robot.hpp
│     └─ floor_robot.hpp
├─ nodes
├─ package.xml
└─ src
    ├─ ariac_competition.cpp
    ├─ ceiling_robot.cpp
    └─ floor_robot.cpp
```

```
group3
├─ .git
│  ├─ COMMIT_EDITMSG
│  ├─ FETCH_HEAD
│  ├─ HEAD
│  ├─ ORIG_HEAD
│  ├─ branches
│  ├─ config
│  ├─ description
│  ├─ hooks
│  │  ├─ applypatch-msg.sample
│  │  ├─ commit-msg.sample
│  │  ├─ fsmonitor-watchman.sample
│  │  ├─ post-update.sample
│  │  ├─ pre-applypatch.sample
│  │  ├─ pre-commit.sample
│  │  ├─ pre-merge-commit.sample
│  │  ├─ pre-push.sample
│  │  ├─ pre-rebase.sample
│  │  ├─ pre-receive.sample
│  │  ├─ prepare-commit-msg.sample
│  │  └─ update.sample
│  ├─ index
│  ├─ info
│  │  └─ exclude
│  ├─ logs
│  │  ├─ HEAD
│  │  └─ refs
│  │     ├─ heads
│  │     │  ├─ RWA1
│  │     │  ├─ RWA2
│  │     │  └─ main
│  │     ├─ remotes
│  │     │  └─ origin
│  │     │     ├─ HEAD
│  │     │     ├─ RWA1
│  │     │     ├─ RWA2
│  │     │     └─ main
│  │     └─ stash
│  ├─ objects
│  │  ├─ 00
│  │  │  └─ 4588edc861c43d481faf10b65d63a4c0f8bdba
│  │  ├─ 01
│  │  │  ├─ b3e16dda93cab84b0c1c89763b09f1338cb6e2
│  │  │  ├─ c1e6785b2d2463cf23f0b8a3729e709d38ac3e
│  │  │  └─ fb661c9267ac43cb2f9e18522c201776819e99
│  │  ├─ 02
│  │  │  ├─ 7fd23eb8011ebc00ef6cc44abdf87761684904
│  │  │  ├─ cc0bb1669e3cb1d84b23cba9fbb34f2cf67021
│  │  │  └─ d49b9b9e06613ccfa246276ef69586b25d7de5
│  │  ├─ 04
│  │  │  ├─ ac62e3a0e865948563d7edbaa255282d685480
│  │  │  └─ b0c252800ada25f70788049e672e4b2e543e1d
│  │  ├─ 05
│  │  │  └─ 266efc44fa5f99232caeceafd39bc427d432f4
│  │  ├─ 06
│  │  │  ├─ 43f90f557332a1e0519cbc17efd0bc8a93acee
│  │  │  └─ 8b94b6580fb284e5fb921b31579a3ca790448d
│  │  ├─ 07
│  │  │  ├─ 1eb4b939426d262bacc96c493465aee905afe5
│  │  │  └─ 8a305e0aa172e25edaa81734f92dc1f15ff023
│  │  ├─ 08
│  │  │  ├─ 60b9020a4d708fc31c9718dab1065430782662
│  │  │  └─ cefa721c6e92c078836a391b147693074f5da7
│  │  ├─ 09
│  │  │  └─ 85ef896152dde3ae1be16aedb8aff5670c084b
│  │  ├─ 0b
│  │  │  ├─ 0ff31c2da3d49b080ce3b8b98670634b25f0e1
│  │  │  └─ cc6576fd36579c10e54da8ef465a2d1e435061
│  │  ├─ 0c
│  │  │  ├─ 674dcd72db336da8bb6754931741ffb2a18dc5
│  │  │  ├─ 8f65883d181dc5497ab346611dd6d85882ae34
│  │  │  ├─ e6523deb86392cdafc22660d2391e8a6bb317b
│  │  │  └─ eac190b067dd037e000fafc88c55b67f4a776b
│  │  ├─ 0e
│  │  │  └─ 8f8f08c33cfc8bceefaf0a0d3a19ca45ca2ab7
│  │  ├─ 0f
│  │  │  ├─ 6c6affd56d4705700cb0e206fb0130f791b3a4
│  │  │  └─ b8ab534786ee990fcf95de1f867db51361af67
│  │  ├─ 10
│  │  │  ├─ 4df5977d8515adbd72412b35bc614dc6787fe5
│  │  │  ├─ 8c569365d71b7cb8ddb413441b3bb7621f8022
│  │  │  └─ b7bfa1488ddfc4d119e1ff5516672b6919a3f0
│  │  ├─ 11
│  │  │  ├─ 451a3a86333ebec4bb11727b17599d7f76bdc7
│  │  │  └─ a7855db5d954a0d5bb158da9f7ca8336cbe4cf
│  │  ├─ 12
│  │  │  ├─ 2a1268717d6602bf8ee4f546a597003f6fe6f8
│  │  │  ├─ 2c0e436a7104ccc992825efeeb1a8e6db59ed6
│  │  │  └─ e7fb9ac12193924d275555962929be67abc84e
│  │  ├─ 13
│  │  │  ├─ 52f4a6a21224a9e58f469fe87860d801b18c62
│  │  │  ├─ 979a3b96eae3b49dcc70e8a94ecd01873e2ae3
│  │  │  └─ c7a9951dcb53cbfa5c7862fd27a472063f3fd1
│  │  ├─ 15
│  │  │  ├─ 3ed5d8276324b6953042719f60e7a9dfcc8e2c
│  │  │  └─ a64d60eb425a6dae2181acba687e0d50aae740
│  │  ├─ 16
│  │  │  └─ 62c0498b0696ad622d8d8c5bf65fd86145be56
│  │  ├─ 18
│  │  │  ├─ bf6a35f03ca7a70977ad56a4eef470732522b8
│  │  │  └─ d06fca32695418ec315350d7cdee85c3ea4d49
│  │  ├─ 19
│  │  │  └─ dd5b42223891434f46263af2e684dcdb81c5b4
│  │  ├─ 1a
│  │  │  ├─ 05b65e55c58a41cbda84963c95d5ea8e699143
│  │  │  └─ ea9ff331c7bf94dea620df2b1c84ff6084ec05
│  │  ├─ 1b
│  │  │  └─ 3316f042210ac491d62ed02f06665315107439
│  │  ├─ 1c
│  │  │  ├─ 565f34e97bb546c033283fa7e50983fd438a39
│  │  │  └─ f583a28f2e6b53cedc7e59f605e0288db82775
│  │  ├─ 1d
│  │  │  ├─ 2cae995e5ae7c67c27fcfcf44c95de27f3e1c1
│  │  │  └─ 4a974c8a844c70559ce91b49d67b0fa4d6b2db
│  │  ├─ 1f
│  │  │  └─ f503cddbb40fcbe19f66a490bad1d11bc0690a
│  │  ├─ 20
│  │  │  ├─ 345b11a87a308f89455058941d71f9bdcf32cc
│  │  │  ├─ 66ee5369e6b6179a31308d63e450f4b36d0508
│  │  │  └─ 91b400ae482061d437eeb1e573292337abcacb
│  │  ├─ 22
│  │  │  ├─ b29673540d173d0542a9d4d55a9539fa93e8ae
│  │  │  └─ c2f01e48b81fe01a49358fe81197a0ee3091e3
│  │  ├─ 24
│  │  │  └─ 46400124b2341b87bdad295bbc9522976bf323
│  │  ├─ 26
│  │  │  ├─ 1e7eb9f107aecfc52bcafc9936f3443cdf4fb7
│  │  │  └─ 65ef42f969c329d608ae142fdec81911a9baad
│  │  ├─ 27
│  │  │  ├─ 187bb044f17b60665b1bcf5883df7438c860a5
│  │  │  └─ fc476c1940f3c5792b54b7078529cf4b24a507
│  │  ├─ 28
│  │  │  ├─ 33a8d802306c407b543d72bf9d49bdedb9cd49
│  │  │  ├─ 4d6c31da65396eb54a23207f17455d6088ca1d
│  │  │  └─ c0ba57b23fe68019a166fbc7b1e6ea5d13eee6
│  │  ├─ 29
│  │  │  ├─ 25988a0633c1c7e9fa00eb0674790a262adc53
│  │  │  └─ 8754510486eef76b13e39d0daa7472dc417633
│  │  ├─ 2b
│  │  │  └─ 5632113cc84ad8dd2f88b014603ed7abbb9177
│  │  ├─ 2e
│  │  │  └─ 6a97703f42d972f65b4bbd42fde84246459138
│  │  ├─ 2f
│  │  │  ├─ 3d6e2cc357e52880e55fd9dc2bdabcae0b4976
│  │  │  ├─ 7ffcd2294ad3fdf82be486316df9b12ef1d724
│  │  │  └─ 8481e1c321bcc3a0e11a3c58edfb4784cd4a28
│  │  ├─ 30
│  │  │  └─ 7e31eae2dc56b10cd362d4f2ecfc224fc8ffe9
│  │  ├─ 32
│  │  │  └─ 55342590f489cb1eb81f6e2345299dd5377e6e
│  │  ├─ 33
│  │  │  ├─ 0331fb86a558eaa2b1a173e1afbfde578f855f
│  │  │  └─ 4913e8c5405f8e5fa4fb73b228094062bbb126
│  │  ├─ 35
│  │  │  ├─ 9cc7b3c29d011d1921848f2d3a39cb3ad0a5c9
│  │  │  └─ d4658f425a1cd56835c1636cdba213375ee32f
│  │  ├─ 36
│  │  │  ├─ 33d9b3f1aaa518eeeb2c3995b5d7b2e33c3ba3
│  │  │  └─ 44a29840d3bbda8b908860eb81fae5db14fbdb
│  │  ├─ 37
│  │  │  ├─ a86e160f7a1ec489aeaba79444336e8c428654
│  │  │  └─ f2bda3f31850e999934aaca135bf6274885e7d
│  │  ├─ 38
│  │  │  ├─ 1b2c098429ae661d943a9cd2de8ee21ad13006
│  │  │  ├─ 8d459844a79807a8052698a7b7cfc942e559c7
│  │  │  └─ e37d9060c38e6beed68915cfa8eecad476df54
│  │  ├─ 39
│  │  │  ├─ 2a9a19a5112c95cf999d6faef218341388645c
│  │  │  ├─ 4d6e2f52b1ee389930b55f83b60e8c2b9f03f0
│  │  │  └─ d803751cc8202dfe5b1dedc9bb4b99599798a0
│  │  ├─ 3a
│  │  │  ├─ 32b403ad54bbaba1bd88f27c32a2f065e2a5d2
│  │  │  ├─ 7e64869e5305380da08aa33cd16d59afa11b83
│  │  │  └─ 8c76eed328012f230cf8504daf449672195da8
│  │  ├─ 3b
│  │  │  └─ dfccf911294dbd49d6d69ae0f257f8e7e1d453
│  │  ├─ 3c
│  │  │  └─ 3c60b823aa9c020a845096ba0cf74f71113938
│  │  ├─ 3d
│  │  │  └─ 7133b9c71569da46d0ff6b0d962aa6469d78bf
│  │  ├─ 3e
│  │  │  └─ 4f382591847ba4a91833aa42ecfa5a20cb0020
│  │  ├─ 40
│  │  │  ├─ 0737c6503dbf10e6de75d248c6e361a92cf1dd
│  │  │  └─ 4ca20c86cf2f424968b2793724f78441f8a958
│  │  ├─ 41
│  │  │  └─ 6937ed1fed323fe5073fbfdcf2f70e82637537
│  │  ├─ 43
│  │  │  └─ ba1d238c43ad8f1d61d062d14a07069d26a8cd
│  │  ├─ 45
│  │  │  └─ 2618cf47142aeb5f0b435af7060f25aaf55143
│  │  ├─ 46
│  │  │  ├─ c4c671aa81f30e740dfb43a80d884d1c50257b
│  │  │  └─ c8eb027bcc0c698b13b9838536c459ebd032b0
│  │  ├─ 48
│  │  │  └─ 830562c53cf2c4582776d60765dace437f7484
│  │  ├─ 49
│  │  │  ├─ 9736fa86524c171411148042c758fb09e61106
│  │  │  └─ a7038eb737f9b63b5b8587367e988b693ee769
│  │  ├─ 4a
│  │  │  └─ 19f9e56fbf0a0a2e7bcef39daadc0adfd71f9b
│  │  ├─ 4b
│  │  │  ├─ 05258a8fbefe360e806c2c536e482a1f43bf46
│  │  │  ├─ 3cacd6068cf240e0057f99a121a1a00e40a40d
│  │  │  ├─ 540beabd0c90b7078f2ae16c31b90715d38cc1
│  │  │  ├─ 79225e30da87aa1a4a1ab6009e8d666d20e4c0
│  │  │  ├─ c5c3e274a2935302dae5d1e61bf3f62ac2f675
│  │  │  └─ e332d4931df40d290a7fbf2a7adb845211509b
│  │  ├─ 4c
│  │  │  ├─ 07612d1bbfee14c6a51a4e4344ee818ba56715
│  │  │  ├─ cbca0353974774d33e5a209216f6264f27df2f
│  │  │  └─ dc8b4928ea849176c3d27d98c76c15e46352fe
│  │  ├─ 4d
│  │  │  └─ 5667490badec34f769dccc1506f2498a8f2a07
│  │  ├─ 4e
│  │  │  ├─ 31196e1a5230e6d0b0266d8aab532d53abea1b
│  │  │  ├─ 4ecfca089a87701b1616462f5f50ff9f8a7fc2
│  │  │  └─ b4496bbdea639fca1c395e9ce65545efd1ca7f
│  │  ├─ 4f
│  │  │  ├─ 398cbaa9ad74309cdcf372865eb691e1236081
│  │  │  ├─ a8412153d9c129fd87570180b53918a69247b5
│  │  │  ├─ c63025521d17df9569b26df96809d085df3763
│  │  │  └─ d669e1ff58ce2ee0ff98eb14d1d2b8c257de1b
│  │  ├─ 50
│  │  │  ├─ 452d6695d53202a721b3c3be5f00bd2451008d
│  │  │  ├─ 9e0cf07abe4974cfcec00465b1b8fe5c0111bc
│  │  │  └─ c156726bb90a155db1edbe7ef0286ea99a910b
│  │  ├─ 52
│  │  │  ├─ 6982c8ac9a51b603fe12470360ce4233ea4949
│  │  │  ├─ 7dda09224447fcf63173bce171453e21ce6883
│  │  │  ├─ d081e71d41f984a795aa04aaa18adf25fec80c
│  │  │  └─ e344471ef4dc824d441cd4f79fe2a18477d95d
│  │  ├─ 53
│  │  │  └─ 6e6596476b7181178b14bc83e59967bbb8bc90
│  │  ├─ 56
│  │  │  └─ a37bd716a9230fc3f0d8cd8b30b5293034cdea
│  │  ├─ 57
│  │  │  ├─ 1b4a5ca57320860b3c859e5585faacf45ee967
│  │  │  └─ ef6788e12516303754b6415d6d4ad22e9d2f8a
│  │  ├─ 58
│  │  │  ├─ 58c9d05f26941a61e55109d8593b75cfe1fae2
│  │  │  ├─ 78116c58bf38088bc653409c12fe311738880d
│  │  │  ├─ 9bc72c40738fe6396ca3a3dd236d0cb3f71e76
│  │  │  ├─ ed2d9e6a2cf287821331abe636c0b7d492802d
│  │  │  └─ ee962a9fbcfb5e404f1ff419ac1c10ba6e9988
│  │  ├─ 59
│  │  │  ├─ 3a0c7e4b6d79d53bac39eebef1abdd89361e7d
│  │  │  ├─ 609062c98c70530195a28ec65ae1a64e34c46f
│  │  │  └─ b395e3dc38c3bde2d4b1f161436d157c0c19c3
│  │  ├─ 5a
│  │  │  ├─ 851656e21e1a3935545afbad50c65a67179bdf
│  │  │  └─ c0c125ac27fb2ca0d573f5c5d32594989b23ce
│  │  ├─ 5b
│  │  │  ├─ 26cac925dfafbf1cb0e648daee96f92135490a
│  │  │  └─ 5c528926af554dcadeaf42b090322425c5c3dd
│  │  ├─ 5c
│  │  │  ├─ 7e63b7cfa9c1526c93064b9b3e4930fbcdd7cf
│  │  │  └─ c7af41716e1d083b987d741780bd0caaa3cb89
│  │  ├─ 5d
│  │  │  ├─ c8a60acbf2e9c085462bf440a4e0434e2e14e8
│  │  │  └─ ef84c810f4cc1b5398918af6799b82e996b6f6
│  │  ├─ 5f
│  │  │  ├─ 3b186fce75006465a3b06072eeadfa332d6b55
│  │  │  ├─ 58cdf8c76c8ca5e2371332ffd648441992a349
│  │  │  ├─ 8f76ac39297a018c37c60b44cce99846689d45
│  │  │  ├─ dfcb28383671d3ce39e777fe911ec477374465
│  │  │  └─ e0e5c8329039b6c9cbf1eedf91e87d5f16f8e2
│  │  ├─ 60
│  │  │  └─ 3a4148700279f643d1f3c7ac8833c175ad4475
│  │  ├─ 61
│  │  │  ├─ 87db8274c53a7a3aacb1470da3f31822592f4d
│  │  │  └─ 93ec4513def06495ea11a2565d26354937ccfe
│  │  ├─ 62
│  │  │  └─ c4daabfd70ab1c5b3e2051d4defb39c43e6be5
│  │  ├─ 64
│  │  │  ├─ 423a4ed5f2069e22396b09a1c3fb8ea99ab534
│  │  │  ├─ 902361aca5e94615cd91f623b31d26119c61d1
│  │  │  └─ a210c5c67b6a0bd4390c3babe0cc5b471dff3a
│  │  ├─ 65
│  │  │  └─ b17f889a3d6c6fa96956444e3bf80189b441e7
│  │  ├─ 66
│  │  │  └─ c600e3dd32c8aa4d5b615a258f521f2dc7b0d7
│  │  ├─ 68
│  │  │  ├─ 064e060a51a89f9fe997ac5b8b225137044b65
│  │  │  └─ fc8e7dca23e6fa57cf7f84aebd1df992ba5e9a
│  │  ├─ 69
│  │  │  ├─ 57a8e6f9b3ece13fb38f74c375f853766750b2
│  │  │  └─ e5108cb586dce610978d132a5368ff181ca58d
│  │  ├─ 6a
│  │  │  ├─ 3b7c7aeb3da9886b7d0c56390073359337537d
│  │  │  └─ 460e4b388116eff14fca1619c85d1f2cc299bd
│  │  ├─ 6b
│  │  │  └─ 986a0ff965b5bdd199987c75f7ce58599120f9
│  │  ├─ 6c
│  │  │  ├─ 9ad5d50fb6268abc1349acd718ce0e8024f01a
│  │  │  └─ abd68aabfe103223bcdcf2069f5eba15994709
│  │  ├─ 6d
│  │  │  ├─ 5b2a075324867636e52856ac94ed872d37bec6
│  │  │  ├─ 882fe26adc3bfe8ea54b4c15ff968fa8d03369
│  │  │  └─ f001355651733ccf718c2c9047a30254587170
│  │  ├─ 70
│  │  │  ├─ 993accdd789cdab0b707ff15513dccc246f1b4
│  │  │  └─ c271faf89b96223d6495df8e8f3bf0362db9b7
│  │  ├─ 71
│  │  │  ├─ 324cf3ed8c5b7283f644d1b9b62d984a3d43d5
│  │  │  └─ ad0be7aa1223765b4804a8a92e8d0345244e7b
│  │  ├─ 72
│  │  │  ├─ 250082444c5a8d8f9b469f0f28ccdaca88013b
│  │  │  └─ dac1c2450614ff9945525171641a5b8a6da4bc
│  │  ├─ 73
│  │  │  ├─ 4c380ba5a47e8201cbc585bbb52611f5f65c2d
│  │  │  ├─ 52f23635c8e49a951dbc19b63454f9ecd399a2
│  │  │  ├─ 696a6251225ce640d359b879242060ca3b332a
│  │  │  └─ 97d1aa4b3e6149a2e28935a6d740c40ae56b65
│  │  ├─ 74
│  │  │  ├─ 347535198ff887a09c7099d604dd425580b4b9
│  │  │  ├─ 4d0ea97868d5a18f9f6501d1eab3066e629608
│  │  │  ├─ ab27c408516e93ed9f5d09d2e5ec1066686a90
│  │  │  └─ bee1b7fab604ed24425bb4a86141df89ec6ff4
│  │  ├─ 75
│  │  │  └─ 79e7197140c5313d2b2260677f30a64754cd95
│  │  ├─ 76
│  │  │  ├─ 3307fee858badbee90f9712f53774d5cb959d2
│  │  │  ├─ 9b2b83859910a28ef63b3461262e40e8bc8e9c
│  │  │  └─ ab76fe2105f4318433b44e0b6e5579c34b8618
│  │  ├─ 77
│  │  │  └─ af86a13e2392b800ffe59fa406a9fc663cc156
│  │  ├─ 78
│  │  │  └─ e9219979a9f46f244b4287a1b2e38a3bffa645
│  │  ├─ 7a
│  │  │  ├─ 116ffadc902856113d914947248b9867c3292c
│  │  │  ├─ b680dcfaf8524249fc1abcac3bbb1873002988
│  │  │  └─ f18a796df3735baa3657f7991fa8617cdf62c2
│  │  ├─ 7b
│  │  │  └─ 9559ce696e4eda2a2e235e9ca79e3c441ee4fb
│  │  ├─ 7c
│  │  │  ├─ 3136e7213ec1412efc7cf4eec239f8834b90ed
│  │  │  ├─ 87b7db5e882f5de63afe0274cc4c468f4bf816
│  │  │  └─ b749efd8334b4dfd6fa468df4528fe384c3d2d
│  │  ├─ 7d
│  │  │  ├─ 612d4fcb086019283d4cd4eaf3ed8ac58eb132
│  │  │  ├─ 8047a16ffbd24ff2172af3e5348c5bce2ebec9
│  │  │  └─ b4c4d2573b5eac488217ddd0388b05d73ca267
│  │  ├─ 81
│  │  │  └─ 429d0603d95e739151a3ce778e7aec3ede9e8f
│  │  ├─ 82
│  │  │  ├─ 07c050bcf5155aa27ce012c3946f4658d0cb0a
│  │  │  ├─ 42d7900e555c1bdb9ce227517f43ab5584b1b2
│  │  │  ├─ e19f5ea953690fd7aa79df9b85aa94d69997a5
│  │  │  ├─ f40a3e76a037b59bc073021f242f58827f3c3f
│  │  │  └─ febc8e5beb6cb59327fed6fbdab299dbb55610
│  │  ├─ 83
│  │  │  ├─ 6c5391b9fb9e0b8889de4a37a5a373686adb9f
│  │  │  └─ ce8de0ab525ad66d5cb82d4fa13c55f12be852
│  │  ├─ 84
│  │  │  └─ 69272e951e6e15c5fbf6dd50ecaf9dbbf9d159
│  │  ├─ 85
│  │  │  └─ 1e923822677bf962a40573af03b34ad2fefe1f
│  │  ├─ 86
│  │  │  ├─ 238281e854b6fb9ede0d629234993e00fbad3b
│  │  │  ├─ 64d6796ac8ce447a8b795587cf51003f5ed2f1
│  │  │  ├─ b46d2ac956e90966c5b6d044c9a427c29ab21a
│  │  │  └─ ceb002138a5d79151cb839f966572b98b27125
│  │  ├─ 87
│  │  │  ├─ 37c5500ce5a6dfe2d65f9fb5a487f7fc617f75
│  │  │  ├─ a2b0d7997a11c0c474b65a03d7d62f1b391019
│  │  │  ├─ ce463ceee9466f72c9b742474619ff99303872
│  │  │  ├─ dbed398246058adec892e2541454445175a49b
│  │  │  └─ f82df1a8db3507f316628ef579fed518eeebea
│  │  ├─ 89
│  │  │  ├─ 40729e1ecc365c3fc64f064e46186e5c9e82bf
│  │  │  ├─ 4bcd0b14c514acdd4500f904b78b98613c95a7
│  │  │  └─ 6f06da97ba6cd040858a702fbeb6def10ee600
│  │  ├─ 8a
│  │  │  └─ 0048c5808b1df3ab07abd3223b4af8a3c2a9c4
│  │  ├─ 8b
│  │  │  └─ d2fabd14f63f28a0a9dbcee17357cb0c2e8300
│  │  ├─ 8d
│  │  │  ├─ 9b09b76aebed2c2ab4c5eeaf1fba81aa1946b6
│  │  │  ├─ ecbc24d136c74c4cea8667962fdcbf4a80b81c
│  │  │  └─ fb3d63fa66dcccbeef1521ff08028ae167067c
│  │  ├─ 8e
│  │  │  └─ 0ce0dcfc1b780a72702e0337c9dbfd98c1e1bd
│  │  ├─ 8f
│  │  │  └─ dd1988d3bee9821956625684a62932344e47c7
│  │  ├─ 90
│  │  │  └─ 2090a8f42bb08480ebe59676766e11ac80c842
│  │  ├─ 91
│  │  │  ├─ 162cda7023b9d49005c3b1960d2da3bb5c5cc1
│  │  │  └─ 8d516f4ee023ba4e5ad746a5fbc2788d11cb63
│  │  ├─ 92
│  │  │  ├─ 2460bc47e289e130a0f145893794efbea3bdc6
│  │  │  ├─ 68f5e96128d8af68aa614dad30d228b3b38bcf
│  │  │  └─ ecb9f01be5e273d1ab131362a935c9c915515e
│  │  ├─ 94
│  │  │  ├─ 6790fd52927cb58460384e65addd85c44ac3b6
│  │  │  └─ e8dbf3e9cc80655fe40dd7a54e0f4589cd02f2
│  │  ├─ 95
│  │  │  ├─ 2ce17c91d6ca24491e27eb672afbb43168bf57
│  │  │  ├─ 54c4f1111e8cf73708c13cc7a3363dde55cd7f
│  │  │  └─ 7aa81bbb2dc45daa69f66ffca1c1454072830f
│  │  ├─ 96
│  │  │  └─ d54e5e0aa293863c5566788a3e1c41ac32c6a8
│  │  ├─ 97
│  │  │  ├─ a3df9afbdea778cf2b4c5546e596f9d092643c
│  │  │  └─ f524e3236e1cc41ab5bf72f4bdcf78480d8467
│  │  ├─ 98
│  │  │  └─ 6a2c29cfa34bb250847dacfc6dbd191d3ca1cb
│  │  ├─ 99
│  │  │  └─ e814fa9ddb12631ea9dbd2bc7a99e132834a4d
│  │  ├─ 9a
│  │  │  ├─ 8a1472251e967da7154fff68a163f11f2af3a7
│  │  │  └─ dfff7b17901f93bed459ede852b7992c6687f8
│  │  ├─ 9b
│  │  │  ├─ 83776f6451e9bbc9a4a56231024b7745e3619a
│  │  │  └─ cd8d66ed9db91b42bfedb1f8547c8da85188c8
│  │  ├─ 9d
│  │  │  └─ 6d36de010f27121f6ffcd8913878827d26c61b
│  │  ├─ 9e
│  │  │  ├─ 00cba7bcfa497b5744475c51cba7fda2537825
│  │  │  ├─ b5281e8d19d69510966b13f980dabfa708c199
│  │  │  └─ c46cdfac493e106fb2f2a110646e4ffbb9035b
│  │  ├─ 9f
│  │  │  ├─ 619cfb8cba946d1795193596ee55df503d9b29
│  │  │  ├─ 6890153da5f3bad254e1deebccc4febe88c662
│  │  │  ├─ 9be06f8a8e41315259eac4990228854f670ee9
│  │  │  ├─ d3e42e4bfc9799fc95b1a36072bad61201bca2
│  │  │  └─ d58dfff258cd7c3de932de795c8c572aec6470
│  │  ├─ a0
│  │  │  ├─ 0157dc84d2999858bb6b2757fbb0343fe48e79
│  │  │  ├─ 3a6b33e2e20efe59b093fc4193c7f97f452430
│  │  │  ├─ 549ab639f9fbb45560ef2ebcbb21e756b8558a
│  │  │  ├─ 9eeb5cf5cc9cfe2f776a13eb7cb03a0d8f3b39
│  │  │  ├─ cd121091c16fd6ad8cad6db5ce3e53c6674f2e
│  │  │  └─ d55be72d6eed52598889695c6e0cc098a525fa
│  │  ├─ a2
│  │  │  └─ 6595dfb7b50832b17e8d7ad2d4248336871a0f
│  │  ├─ a3
│  │  │  └─ 4a48ecfa934c6af70ca8e7473770aacc3a5dc9
│  │  ├─ a4
│  │  │  └─ 276f3725865ef1723b0908b9bbb6a6899b0946
│  │  ├─ a5
│  │  │  ├─ 32fa272d67259cf1c0bba780535351f7043054
│  │  │  └─ 6a20e8f0900de9702af7f64780056a5e504371
│  │  ├─ a6
│  │  │  ├─ 212c1a5ee550fe7d0018f11dc47110b657da87
│  │  │  ├─ 4570f0ecf425a01d059af394a202ff2e6b3ffb
│  │  │  ├─ cdf862b3ff15b1237391c2fe5be93b0faa9d42
│  │  │  └─ e853f8e4ebaee776f360759d53c02b09355825
│  │  ├─ a8
│  │  │  └─ d145d19ba5c901dd9a2253b9721327c9a8e632
│  │  ├─ a9
│  │  │  └─ f91b2999e0493edcf9e08246c4b6301bad5e0b
│  │  ├─ aa
│  │  │  ├─ 01a4b0dc7147eb93af77efa3d332d38c1e4c78
│  │  │  ├─ 1146d3d450be2ed9704f0a8123d3ebac738a0d
│  │  │  └─ a2dadd10e902e50582afd0f40db36c797bd265
│  │  ├─ ab
│  │  │  ├─ 705e2e124f60ecba67bb663b14c43896e4058d
│  │  │  ├─ 77aa0d1b6cba81a093fc1425d7fd1ecf3c4ec8
│  │  │  └─ eb3dd0991ea002880a88daff4c0de6edfe1167
│  │  ├─ ac
│  │  │  └─ ead71cf5c640a7b4ff3a9f842b7729088c4245
│  │  ├─ ad
│  │  │  └─ c5f328ef33ff6ff00c50ea2af46f7153cdc1d9
│  │  ├─ ae
│  │  │  └─ 77a525b41ed1be12719b9af97ee8231be4ebb6
│  │  ├─ af
│  │  │  ├─ 82f7f0f18a1cea30160f63d23732fd84923601
│  │  │  └─ ec68d732838ea0e9491f37864ce125cf8e7847
│  │  ├─ b0
│  │  │  └─ a6c29145fb99ec14bdfa72ae415239302fd660
│  │  ├─ b1
│  │  │  ├─ 556df5fb7d651b806e58206fff58210ebc74a1
│  │  │  └─ d6f9c42f0483077911b889751711a5c3149f94
│  │  ├─ b3
│  │  │  ├─ 63652c36a9aac2fdd71e0ed1cef2adef908325
│  │  │  ├─ 6e459cb2a36491dfeafc820bd6e6325e65d171
│  │  │  └─ 972bde2858960ccac2b113f4c3772f25454c61
│  │  ├─ b4
│  │  │  ├─ 3350325e68cbe2f39c8c838f9d42ab07197d74
│  │  │  ├─ a79baaf421df92a2017b6435b6dbcc5d75053f
│  │  │  ├─ d606f565ec57747805a06841ace6f99cb3c35a
│  │  │  └─ d784ec8a2e6b8c1b9123e100472168560b4b11
│  │  ├─ b5
│  │  │  ├─ 14c85ba1d2227eb0d3eb27161623c75dda7bdd
│  │  │  └─ bed1cd75a64debb653020f517a4ac4bcea5a2c
│  │  ├─ b6
│  │  │  ├─ 0e8dc8c5900fd09a74af392f2e224e5e92c211
│  │  │  └─ 3c8f44d5c4fc1dd36e5a23dd83411da7414197
│  │  ├─ b7
│  │  │  ├─ b953e4a64909e3cf58fed253f3a605fab19590
│  │  │  └─ ffed2dc1915a64eecf2a2a77efc5fddcef7da3
│  │  ├─ b8
│  │  │  ├─ 335df0d293c20abfcd88bbe239d245782d6655
│  │  │  ├─ 5ffb1139e6329850582727c57f9eac71d1296b
│  │  │  ├─ 88a3c8f2b8a48cf2186560173e5b1f2f3b8e5f
│  │  │  └─ efc6fb5afb9103047485b5f46c6280b05c0c50
│  │  ├─ b9
│  │  │  ├─ 39dec035bcfee7ad0cde6b08fe75c93ce120f8
│  │  │  └─ d674f1324c29cda05e924ddc29ac24ba190cc0
│  │  ├─ bb
│  │  │  ├─ 785506e4efd7dcc2cad020c8f2071a0ac9402a
│  │  │  ├─ 99f5a484990d6d6c2f8b49b6f6aaf1bc0839e7
│  │  │  └─ b927ce24721a428a3acdba6f2be5b0d679fd30
│  │  ├─ bc
│  │  │  └─ b2196f2b8cd2f9ad6102f976f2b25d69a79553
│  │  ├─ be
│  │  │  └─ 43bc3ccdec8d61b37c63f5acb0dd9e3bb83803
│  │  ├─ bf
│  │  │  └─ 8fd18ab3bb42c449033c7fb4238b4db9f2cfe3
│  │  ├─ c1
│  │  │  ├─ 34a1b30d54016ef4110106c7178e1d577b3474
│  │  │  └─ cb6f542cb1d3b2ea536eddfd16b5ca0d08a1fc
│  │  ├─ c2
│  │  │  ├─ e5bd992801bc84c6049a3cbf727ecea2b24bd8
│  │  │  └─ ed5573a0e585e9cf58a288675c533985d210c6
│  │  ├─ c4
│  │  │  └─ 44fe7070d5eaee359e653af01a028497f660a6
│  │  ├─ c5
│  │  │  └─ 542159f041aec292535e206fa5d3be32621999
│  │  ├─ c6
│  │  │  ├─ f5697caafd897b9f505f15ad71daaaf8d59b26
│  │  │  └─ fb1a30146797c56181e53b4b409d092330c70e
│  │  ├─ c7
│  │  │  ├─ 3094c193c0b5984f416a848642e1e298285a73
│  │  │  └─ b8548465e5d2ee710a51034d25f65e7790089c
│  │  ├─ c8
│  │  │  └─ ab9d652467f8a66f8ef7966f23a2694a5720cb
│  │  ├─ c9
│  │  │  └─ 3929802b3fcac8ce54aa7994d8ac2f02477dcd
│  │  ├─ ca
│  │  │  └─ f13a0b5fe3d18acb7860983994c59a53aed452
│  │  ├─ cb
│  │  │  └─ 07cadd348fd1ad343c2fd3c4cb6cb66f7772c2
│  │  ├─ cc
│  │  │  └─ 4ae8430785a769164e14b0f95383d9862f0737
│  │  ├─ cd
│  │  │  ├─ 56197fa07edc16923a5692caabd72db4c642e0
│  │  │  └─ b469d69028e2c0507e92118d3d9560febb9ffe
│  │  ├─ ce
│  │  │  └─ 88f6b3821218375869a3536455d1d9f61210ce
│  │  ├─ d0
│  │  │  └─ 948462628e3cf229621c9b522c17a818ae5de9
│  │  ├─ d1
│  │  │  ├─ 7fa26b78d5ebed5e1dd1bccbb74815e5ea5e20
│  │  │  ├─ cd5bfab39efbe2f3f5ddd5d36b2a6c3481d112
│  │  │  └─ f0401186cf544742fdae71e3c8ff34351eb1dd
│  │  ├─ d3
│  │  │  ├─ 228471d61ac013cdd91799619fb18f1eec2c54
│  │  │  ├─ 8b7c760edd41ca7c528ca347a49f2e7f436d45
│  │  │  ├─ b5e002eb8ade06b00d9768a28fdf0bfd6aae48
│  │  │  └─ caadab79d65bb372bfe560fc281488a9384c84
│  │  ├─ d4
│  │  │  ├─ 70cf1ca6c1dad453e153195a506e97866762e9
│  │  │  └─ 8892efff021f427a783de955fbb5677ea0d6dd
│  │  ├─ d5
│  │  │  └─ b9e82319c5e6ad69ba93918857c583928a1bd6
│  │  ├─ d6
│  │  │  ├─ 1d439b9e264fe57f53a43c77692adb5261dede
│  │  │  ├─ 5c99676f9c330dbca9da975bf6002b8b7847e4
│  │  │  └─ dc93924764cabe16ca244b6e87e8099b81219a
│  │  ├─ d7
│  │  │  └─ 9fea2c2e8c031c4724903ab660a7b118be513d
│  │  ├─ d8
│  │  │  ├─ 98357febcaae2c349b994574bfe1b167c67470
│  │  │  └─ dd3892499ae96e02fc19a276d1d8c53a65e063
│  │  ├─ d9
│  │  │  ├─ 40bf2cc591d972481eedd0608682afe93d43b8
│  │  │  └─ b7a9171ef750960d2181892cb67e32bb01bcb0
│  │  ├─ da
│  │  │  ├─ 5ac6c3415f2ec7a7f5d16703b76e6896d5a927
│  │  │  ├─ 8b4f440f7087c1b668084d977c9b9daab76a6e
│  │  │  └─ ade6606281a6afa36cdd32d7bc3cfdaf31773e
│  │  ├─ db
│  │  │  └─ 02840a48890cfa15de479e29618f9ac8421686
│  │  ├─ dc
│  │  │  ├─ 9212f1d7e0fbd85802803e93a2b60a6c49b051
│  │  │  └─ e790a6a48623b789a59390efbe74655cbebb60
│  │  ├─ dd
│  │  │  └─ 637162e16e03f59be59cf9a2fd325294e7dc0f
│  │  ├─ de
│  │  │  └─ 244b0a422fdc06870176bca3daf8717f6c9b36
│  │  ├─ df
│  │  │  ├─ bab306834b7d0ff85b245a2f17c2000a7bc6a5
│  │  │  ├─ e116c3be693d8ed9a5dd6267d9f3593075d5e5
│  │  │  └─ e3f5a75b826316979018b948a0a5dd40fa1bea
│  │  ├─ e1
│  │  │  └─ 7496b1d787fbd710abb3a1dcb54faef900251b
│  │  ├─ e2
│  │  │  └─ 7d68d338aedbf80008ed40abf138957883067e
│  │  ├─ e3
│  │  │  ├─ 1dd92edc9c07b20f00907ae804767cb8beeb4c
│  │  │  ├─ 268a15b7555c8e55e46b625dd5f850aa51b8b8
│  │  │  └─ 8401b04c401c9ca011d933e5ce47b2441d6db7
│  │  ├─ e4
│  │  │  ├─ 34db55b78e19fc3e3f15df08d3db359314b67a
│  │  │  ├─ 963875693df6f17ff9a3f083975f30dd321c9b
│  │  │  └─ 9c5f1e6a632c421e9744ee10403f9562583e27
│  │  ├─ e6
│  │  │  └─ 9de29bb2d1d6434b8b29ae775ad8c2e48c5391
│  │  ├─ e7
│  │  │  └─ 51597e5ff3bac4ad1444871b300a0daa9db0cc
│  │  ├─ e8
│  │  │  ├─ 6e070045a5bad7f3393a4b8845f441f28d3879
│  │  │  └─ 82674c751eb7e1b43c79991cd3cb477833a7e9
│  │  ├─ e9
│  │  │  └─ 18668051ac86ae44fbd50e0db31490c3718fb4
│  │  ├─ ea
│  │  │  ├─ 39ec2ee4719350b1cb4dbaccba2bddfe8e7be9
│  │  │  ├─ 6e8bd99f483b8f70ce47b0698fa2e8d4010274
│  │  │  ├─ 853f4144123cf7f1308ffbdeeec9c742ca58d6
│  │  │  ├─ a1f78ef4c9d4ec6b481b77fe65d67ab580db7d
│  │  │  ├─ b53e1b005795fdbc104e618b7626d7c6d2e622
│  │  │  └─ f952dff8b290e99682c8068ba64512865676e9
│  │  ├─ eb
│  │  │  └─ e7767f45dd7efd89e7b3fd7c7ace9457d0da65
│  │  ├─ ed
│  │  │  └─ e7a292bcf8dbcfbba641b5a26eaee72250de8f
│  │  ├─ ef
│  │  │  ├─ 3d7180b7e703afe4bd31ab7350fd79b100a029
│  │  │  └─ bbc4649f3aff097368280087e7ec92714f22b1
│  │  ├─ f0
│  │  │  └─ a85875b6c9c48cc24501d8d93a4380080fc947
│  │  ├─ f2
│  │  │  ├─ 356d0d5950ed9c7da7a92f6148549f5ebafa73
│  │  │  └─ 6ff2b9087ca930f6d1b77d6dd9c3c63af6b3a1
│  │  ├─ f3
│  │  │  ├─ 4eddeb519594319ef2c57c4ccff070455a3372
│  │  │  └─ e4d8185312ea47a61c9eef9637c3ea3bfa11ac
│  │  ├─ f4
│  │  │  └─ f8c61854a17e5378d1fa5bb013de1e74857d59
│  │  ├─ f5
│  │  │  ├─ 296f60523f9f724d6fabda47cb5286f08eda6f
│  │  │  └─ 8fb1c6f8d5ec5a7b4fdabfa8b8986444aa2ad7
│  │  ├─ f6
│  │  │  ├─ 33f95fb2585d83a3b8183f13f3232000ab296e
│  │  │  ├─ 48c98204fde2ac596271385292c8d4341fcf3a
│  │  │  └─ 60185d9ad2b13022cc1bbd8193ddb2bd199b7b
│  │  ├─ f7
│  │  │  ├─ 5561e355df1f3f8349d3e43e63c0848ddbfaf8
│  │  │  ├─ 889ba1a3093b6ddffa46531eafdf3e594c5400
│  │  │  ├─ 902dfe1836c5a83bdc5c118972577a10328420
│  │  │  └─ ef887042fc4c02cdab831d29c1923c014d0364
│  │  ├─ f8
│  │  │  ├─ 65a57ce12498081a7382c1536054490b18631a
│  │  │  └─ 786e8e8f51e42eb8ce64df4cffb9f4b698809f
│  │  ├─ f9
│  │  │  ├─ 09d42e4482b4c897cb51c395cc8b4d7bf6566e
│  │  │  └─ 09d93c8cd52f15213f2c916b26a5c014aa27c3
│  │  ├─ fa
│  │  │  ├─ 1f06c529c67edbabb846d92bfeccac25e72d12
│  │  │  └─ 644c580928a6f29a51c9ad813505d98b38a871
│  │  ├─ fb
│  │  │  └─ bbc2a6f175055b5f67bd7acce1cc614075f268
│  │  ├─ fd
│  │  │  └─ a8dee61779d200ebb0ce4a53e76348f78f9436
│  │  ├─ fe
│  │  │  ├─ 3fbc64ed28c14d660deefbfd1f2fc2fa42c67d
│  │  │  ├─ 4974cc34245700b48f4b64be2a011765078b56
│  │  │  └─ c4282fcf6d70fee97266ecd11c8a4d8166174f
│  │  ├─ info
│  │  └─ pack
│  ├─ packed-refs
│  └─ refs
│     ├─ heads
│     │  ├─ RWA1
│     │  ├─ RWA2
│     │  └─ main
│     ├─ remotes
│     │  └─ origin
│     │     ├─ HEAD
│     │     ├─ RWA1
│     │     ├─ RWA2
│     │     └─ main
│     ├─ stash
│     └─ tags
├─ .gitignore
├─ CMakeLists.txt
├─ LICENSE.md
├─ README.md
├─ document
│  ├─ Activity_Diagram_v1.jpg
│  ├─ Class_Diagram_v1.jpg
│  └─ instructions.txt
├─ group3
│  └─ __init__py
├─ include
│  └─ group3
│     ├─ ariac_competition.hpp
│     ├─ ceiling_robot.hpp
│     └─ floor_robot.hpp
├─ nodes
│  └─ .placeholder
├─ package.xml
└─ src
   ├─ ariac_competition.cpp
   ├─ ceiling_robot.cpp
   └─ floor_robot.cpp

```