# 作业与交付成品

这里长期存放准备提交的报告、视频、源码快照和打包 ZIP。指南进入 Git，个人作业正文、成员信息、视频和二进制文件保留本机，不随 git push 上传。

第二次作业历史材料见[作业二](assignment-02/README.md)。

每项作业一个 `assignment-NN/`，例如[第三次作业](assignment-03/README.md)。每次正式修订创建 `YYYY-MM-DD-rNN/`，日期是成品定版日期，修订从 r01 起；不要使用 final、latest、最终版2 等相互竞争的目录名。

```text
deliverables/
  README.md
  assignment-03/
    README.md
    2026-10-02-r01/
      files/                  报告、附件、源码快照与提交说明
      assignment-03_2026-10-02-r01_693c490_submission.zip
      manifest.json           来源提交、校验值及准备/提交状态
    archive/                  早期包、旧打包工具和未采用的报告稿
```

ZIP 使用 `assignment-NN_YYYY-MM-DD-rNN_<commit7>_submission.zip`；没有源码时可省略 commit7。课程要求特殊外部文件名时，提交前按要求改名，并在 manifest 记录，不复制出多个“最终包”。

已定版内容不随主线升级而重打。更改报告或替换视频时另建新日期/修订目录，注明来源与证据。manifest 记录 prepared、submitted 等事实；只有获得实际提交回执才标为 submitted。

运行日志和中间文件放 outputs；保留的原始实验和验收证据放 local_artifacts。详见[工作区规则](../docs/workspace.md)。
