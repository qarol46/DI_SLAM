#include <iostream>
#include <chrono>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

int main(int argc, char** argv) {
    std::cout << "Программа запущена" << std::endl;
    
    // 1. Определяем путь к изображению
    std::string imagePath = "../test_rgb_frames/image.jpg"; 
    
    if (argc > 1) {
        imagePath = argv[1];
    }

    std::cout << "Загрузка изображения: " << imagePath << std::endl;

    // 2. Загрузка изображения
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);

    if (image.empty()) {
        std::cerr << "Ошибка: не удалось загрузить изображение" << std::endl;
        return -1;
    }

    std::cout << "Изображение загружено. Размер: " 
              << image.cols << "x" << image.rows << std::endl;

    // 3. Настройка параметров ORB для SLAM
    int numFeatures = 500;  // Количество ключевых точек (оптимально для SLAM)
    
    cv::Ptr<cv::ORB> orb = cv::ORB::create(
        numFeatures      // nfeatures - максимальное количество точек
    );

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    // 4. Поиск ключевых точек с замером времени
    auto start_time = std::chrono::high_resolution_clock::now();
    orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    std::chrono::duration<double, std::milli> time_taken = end_time - start_time;

    // 5. Вывод результатов
    std::cout << "\n--- Результаты ---" << std::endl;
    std::cout << "Найдено ключевых точек: " << keypoints.size() << std::endl;
    std::cout << "Дескрипторы: " << descriptors.rows << "x" << descriptors.cols << std::endl;
    std::cout << "Время: " << time_taken.count() << " мс" << std::endl;

    // 6. Сохранение изображения с ключевыми точками в исходную папку
    cv::Mat imageWithKeypoints;
    cv::drawKeypoints(image, keypoints, imageWithKeypoints, 
                      cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    // Формируем путь для сохранения в папке test_rgb_frames
    std::string outputDir = "../test_rgb_frames/";
    std::string outputPath = outputDir + "orb_keypoints_result.jpg";
    
    cv::imwrite(outputPath, imageWithKeypoints);
    std::cout << "\nИзображение с ключевыми точками сохранено в: " << outputPath << std::endl;

    return 0;
}