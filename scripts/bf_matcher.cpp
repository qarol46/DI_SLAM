#include <iostream>
#include <chrono>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

int main(int argc, char** argv) {
    std::cout << "Программа запущена" << std::endl;
    
    // 1. Определяем пути к двум изображениям
    std::string imagePath1 = "../test_rgb_frames/image1.jpg";
    std::string imagePath2 = "../test_rgb_frames/image2.jpg";
    
    if (argc > 2) {
        imagePath1 = argv[1];
        imagePath2 = argv[2];
    }

    std::cout << "Загрузка первого изображения: " << imagePath1 << std::endl;
    std::cout << "Загрузка второго изображения: " << imagePath2 << std::endl;

    // 2. Загрузка изображений
    cv::Mat image1 = cv::imread(imagePath1, cv::IMREAD_GRAYSCALE);
    cv::Mat image2 = cv::imread(imagePath2, cv::IMREAD_GRAYSCALE);

    if (image1.empty()) {
        std::cerr << "Ошибка: не удалось загрузить первое изображение" << std::endl;
        return -1;
    }
    if (image2.empty()) {
        std::cerr << "Ошибка: не удалось загрузить второе изображение" << std::endl;
        return -1;
    }

    std::cout << "Первое изображение: " << image1.cols << "x" << image1.rows << std::endl;
    std::cout << "Второе изображение: " << image2.cols << "x" << image2.rows << std::endl;

    // 3. Настройка параметров ORB для SLAM
    int numFeatures = 300;
    
    cv::Ptr<cv::ORB> orb = cv::ORB::create(numFeatures);

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;

    // 4. Поиск ключевых точек на обоих изображениях
    auto start_time = std::chrono::high_resolution_clock::now();
    
    orb->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
    orb->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_taken = end_time - start_time;

    std::cout << "\n--- Ключевые точки ---" << std::endl;
    std::cout << "Кадр 1: " << keypoints1.size() << " точек" << std::endl;
    std::cout << "Кадр 2: " << keypoints2.size() << " точек" << std::endl;
    std::cout << "Время детекции: " << time_taken.count() << " мс" << std::endl;

    // 5. Матчинг дескрипторов
    // ORB использует бинарные дескрипторы, поэтому NORM_HAMMING
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knnMatches;
    
    // Используем KNN matching с k=2 для Lowe's ratio test
    matcher.knnMatch(descriptors1, descriptors2, knnMatches, 2);

    // 6. Фильтрация совпадений (Lowe's ratio test)
    std::vector<cv::DMatch> goodMatches;
    float ratioThreshold = 0.75f;  // Стандартное значение Lowe
    
    for (const auto& matchPair : knnMatches) {
        if (matchPair.size() == 2 && matchPair[0].distance < ratioThreshold * matchPair[1].distance) {
            goodMatches.push_back(matchPair[0]);
        }
    }

    std::cout << "\n--- Совпадения ---" << std::endl;
    std::cout << "Всего KNN совпадений: " << knnMatches.size() << std::endl;
    std::cout << "Хороших совпадений (после фильтрации): " << goodMatches.size() << std::endl;

    // 7. Вычисление смещений
    if (!goodMatches.empty()) {
        std::vector<float> dx_values, dy_values;
        
        for (const auto& match : goodMatches) {
            cv::Point2f pt1 = keypoints1[match.queryIdx].pt;
            cv::Point2f pt2 = keypoints2[match.trainIdx].pt;
            
            float dx = pt2.x - pt1.x;
            float dy = pt2.y - pt1.y;
            
            dx_values.push_back(dx);
            dy_values.push_back(dy);
        }

        // Статистика смещений
        float sum_dx = 0, sum_dy = 0;
        for (size_t i = 0; i < dx_values.size(); i++) {
            sum_dx += dx_values[i];
            sum_dy += dy_values[i];
        }
        
        float mean_dx = sum_dx / dx_values.size();
        float mean_dy = sum_dy / dy_values.size();

        std::cout << "\n--- Смещения ---" << std::endl;
        std::cout << "Среднее смещение по X: " << mean_dx << " пикселей" << std::endl;
        std::cout << "Среднее смещение по Y: " << mean_dy << " пикселей" << std::endl;
        std::cout << "Общее среднее смещение: " 
                  << std::sqrt(mean_dx * mean_dx + mean_dy * mean_dy) << " пикселей" << std::endl;
    }

    // 8. Визуализация: два кадра рядом с линиями соответствий
    cv::Mat combinedImage;
    
    // Создаем изображение с двумя кадрами рядом
    cv::hconcat(image1, image2, combinedImage);
    
    // Конвертируем в цветное для рисования линий
    cv::cvtColor(combinedImage, combinedImage, cv::COLOR_GRAY2BGR);

    // Рисуем ключевые точки и линии соответствий
    for (const auto& match : goodMatches) {
        cv::Point2f pt1 = keypoints1[match.queryIdx].pt;
        cv::Point2f pt2 = keypoints2[match.trainIdx].pt;
        
        // Смещаем точку второго кадра вправо
        cv::Point2f pt2_shifted(pt2.x + image1.cols, pt2.y);
        
        // Рисуем ключевые точки
        cv::circle(combinedImage, pt1, 4, cv::Scalar(0, 255, 0), 2);
        cv::circle(combinedImage, pt2_shifted, 4, cv::Scalar(255, 0, 0), 2);
        
        // Рисуем линию соответствия
        cv::line(combinedImage, pt1, pt2_shifted, cv::Scalar(0, 255, 255), 1);
    }

    // 9. Сохранение результата
    std::string outputPath = "../test_rgb_frames/orb_matches_result.jpg";
    cv::imwrite(outputPath, combinedImage);
    
    std::cout << "\nИзображение с соответствиями сохранено в: " << outputPath << std::endl;

    return 0;
}