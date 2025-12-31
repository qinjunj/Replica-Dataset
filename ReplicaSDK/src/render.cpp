// Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
#include <EGL.h>
#include <PTexLib.h>
#include <pangolin/image/image_convert.h>
#include <pangolin/pangolin.h>

#include "GLCheck.h"
#include "MirrorRenderer.h"

#include <fstream>
#include <iomanip>
#include <cmath>

const double kPi = 3.14159265358979323846;

int main(int argc, char* argv[]) {
  ASSERT(argc == 3 || argc == 4, "Usage: ./ReplicaRenderer mesh.ply /path/to/atlases [mirrorFile]");

  const std::string meshFile(argv[1]);
  const std::string atlasFolder(argv[2]);
  ASSERT(pangolin::FileExists(meshFile));
  ASSERT(pangolin::FileExists(atlasFolder));

  std::string surfaceFile;
  if (argc == 4) {
    surfaceFile = std::string(argv[3]);
    ASSERT(pangolin::FileExists(surfaceFile));
  }

  const int width = 1440;
  const int height = 1600;
  bool renderDepth = true;
  float depthScale = 65535.0f * 0.1f;

  // Setup EGL
  EGLCtx egl;

  egl.PrintInformation();
  
  if(!checkGLVersion()) {
    return 1;
  }

  //Don't draw backfaces
  const GLenum frontFace = GL_CCW;
  glFrontFace(frontFace);

  // Setup a framebuffer
  pangolin::GlTexture render(width, height);
  pangolin::GlRenderBuffer renderBuffer(width, height);
  pangolin::GlFramebuffer frameBuffer(render, renderBuffer);

  pangolin::GlTexture depthTexture(width, height, GL_R32F, false, 0, GL_RED, GL_FLOAT, 0);
  pangolin::GlFramebuffer depthFrameBuffer(depthTexture, renderBuffer);

  // Setup a camera
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrixRDF_BottomLeft(
          width,
          height,
          width / 2.0f,
          width / 2.0f,
          (width - 1.0f) / 2.0f,
          (height - 1.0f) / 2.0f,
          0.1f,
          100.0f),
  pangolin::ModelViewLookAtRDF(0, 1.5, 4, 0, 1.5, 3, 0, 1, 0));

  // load mirrors
  std::vector<MirrorSurface> mirrors;
  if (surfaceFile.length()) {
    std::ifstream file(surfaceFile);
    picojson::value json;
    picojson::parse(json, file);

    for (size_t i = 0; i < json.size(); i++) {
      mirrors.emplace_back(json[i]);
    }
    std::cout << "Loaded " << mirrors.size() << " mirrors" << std::endl;
  }

  const std::string shadir = STR(SHADER_DIR);
  MirrorRenderer mirrorRenderer(mirrors, width, height, shadir);

  // load mesh and textures
  PTexMesh ptexMesh(meshFile, atlasFolder);

  pangolin::ManagedImage<Eigen::Matrix<uint8_t, 3, 1>> image(width, height);
  pangolin::ManagedImage<float> depthImage(width, height);
  pangolin::ManagedImage<uint16_t> depthImageInt(width, height);

  pangolin::Var<float> exposure("ui.Exposure", 0.01, 0.0f, 0.1f);
  ptexMesh.SetExposure(exposure);

  // Render some frames
  const size_t numFrames = 450;
  // open trajectory file
  std::ofstream trajFile("trajectory.txt");
  trajFile << std::setprecision(8) << std::fixed;
  for (size_t i = 0; i < numFrames; i++) {
    std::cout << "\rRendering frame " << i + 1 << "/" << numFrames << "... ";
    std::cout.flush();

    // --- NEW: "walking and looking around" camera motion  for apartment 0---
    // double t = static_cast<double>(i) / std::max<size_t>(numFrames - 1, 1);

    // // Walk forward along -Z, with slight side-to-side and bobbing in Y.
    // double pathLen     = 3.0;          // meters
    // double baseHeight  = 1.5;          // eye height
    // double bobAmp      = 0.05;         // head bob amplitude
    // double strafeAmp   = 0.2;          // left-right sway
    // double yawAmpDeg   = 20.0;         // +/- 20 degrees yaw

    // double x = strafeAmp * std::sin(2.0 * kPi * t);
    // double y = baseHeight + bobAmp * std::sin(4.0 * kPi * t);
    // double z = 4.0 - pathLen * t;      // start at z=4, walk toward z=1

    // double yawRad = (yawAmpDeg * kPi / 180.0) * std::sin(2.0 * kPi * t);

    // // Forward vector from yaw (in RDF coords)
    // Eigen::Vector3d cam_pos(x, y, z);
    // Eigen::Vector3d forward(std::sin(yawRad), 0.0, -std::cos(yawRad));
    // Eigen::Vector3d look_at = cam_pos + forward;

    // Eigen::Matrix4d T_view = pangolin::ModelViewLookAtRDF(
    //     cam_pos.x(), cam_pos.y(), cam_pos.z(),
    //     look_at.x(), look_at.y(), look_at.z(),
    //     0.0, 1.0, 0.0);

    // s_cam.SetModelViewMatrix(T_view);
    // --- END NEW MOTION ---

    // --- Circular Motion ---
    double t = static_cast<double>(i) / std::max<size_t>(numFrames - 1, 1);

    // Parameters
    double radius      = 1.0;    // meter radius of the walking circle
    double baseHeight  = 0.3;    // eye height (z)
    double bobAmp      = 0.1;   // vertical head bob amplitude (m)
    double yawOscDeg   = 50.0;   // extra yaw "look around" amplitude

    // Walk in a circle in the x–y plane (z-up world)
    double angle = 2.0 * kPi * t;  // one full loop over numFrames

    double x = radius * std::cos(angle);
    double y = radius * std::sin(angle) + t - 1.0;
    double z = baseHeight + bobAmp * std::sin(4.0 * kPi * t);  // head bob

    // Base yaw: face roughly along the walking direction (tangent to circle)
    double baseYaw = angle + kPi / 2.0;  // tangent to circle

    // Add a small "look around" oscillation
    double yawOscRad = (yawOscDeg * kPi / 180.0) * std::sin(2.0 * kPi * t);
    double yaw = baseYaw + yawOscRad;

    // Forward direction in x–y plane (horizontal look)
    Eigen::Vector3d cam_pos(x, y, z);
    Eigen::Vector3d forward(std::cos(yaw), std::sin(yaw), 0.0);
    Eigen::Vector3d look_at = cam_pos + forward;

    // z-up world: up vector is (0, 0, 1)
    Eigen::Matrix4d T_view = pangolin::ModelViewLookAtRDF(
        cam_pos.x(),  cam_pos.y(),  cam_pos.z(),
        look_at.x(),  look_at.y(),  look_at.z(),
        0.0,          0.0,          1.0);

    s_cam.SetModelViewMatrix(T_view);
    // --- End Motion for office 0 ---

    // --- log pose for this frame (view matrix: world -> camera) ---
    trajFile << i;
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        trajFile << " " << T_view(r, c);
      }
    }
    trajFile << "\n";

    // Render
    frameBuffer.Bind();
    glPushAttrib(GL_VIEWPORT_BIT);
    glViewport(0, 0, width, height);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    glEnable(GL_CULL_FACE);

    ptexMesh.Render(s_cam);

    glDisable(GL_CULL_FACE);

    glPopAttrib(); //GL_VIEWPORT_BIT
    frameBuffer.Unbind();

    for (size_t i = 0; i < mirrors.size(); i++) {
      MirrorSurface& mirror = mirrors[i];
      // capture reflections
      mirrorRenderer.CaptureReflection(mirror, ptexMesh, s_cam, frontFace);

      frameBuffer.Bind();
      glPushAttrib(GL_VIEWPORT_BIT);
      glViewport(0, 0, width, height);

      // render mirror
      mirrorRenderer.Render(mirror, mirrorRenderer.GetMaskTexture(i), s_cam);

      glPopAttrib(); //GL_VIEWPORT_BIT
      frameBuffer.Unbind();
    }

    // Download and save
    render.Download(image.ptr, GL_RGB, GL_UNSIGNED_BYTE);

    char filename[1000];
    snprintf(filename, 1000, "frame%06zu.png", i);

    pangolin::SaveImage(
        image.UnsafeReinterpret<uint8_t>(),
        pangolin::PixelFormatFromString("RGB24"),
        std::string(filename));

    if (renderDepth) {
      // render depth
      depthFrameBuffer.Bind();
      glPushAttrib(GL_VIEWPORT_BIT);
      glViewport(0, 0, width, height);
      glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

      glEnable(GL_CULL_FACE);

      ptexMesh.RenderDepth(s_cam, depthScale);

      glDisable(GL_CULL_FACE);

      glPopAttrib(); //GL_VIEWPORT_BIT
      depthFrameBuffer.Unbind();

      depthTexture.Download(depthImage.ptr, GL_RED, GL_FLOAT);

      // convert to 16-bit int
      for(size_t i = 0; i < depthImage.Area(); i++)
          depthImageInt[i] = static_cast<uint16_t>(depthImage[i] + 0.5f);

      snprintf(filename, 1000, "depth%06zu.png", i);
      pangolin::SaveImage(
          depthImageInt.UnsafeReinterpret<uint8_t>(),
          pangolin::PixelFormatFromString("GRAY16LE"),
          std::string(filename), true, 34.0f);
    }
  }
  std::cout << "\rRendering frame " << numFrames << "/" << numFrames << "... done" << std::endl;

  // close trajectory file
  trajFile.close();

  return 0;
}

