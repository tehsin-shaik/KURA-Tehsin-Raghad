#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace booster_vision {

class ColorClassifier {
public:
    struct Prediction {
        std::string label;
        float confidence = 0.0f;
        bool valid = false;
    };

    ColorClassifier() = default;
    ~ColorClassifier() = default;

    void Init(const YAML::Node &node) {
        enabled_ = node["enabled"] ? node["enabled"].as<bool>() : true;
        confidence_threshold_ = node["confidence_threshold"] ? node["confidence_threshold"].as<float>() : 0.60f;
        input_width_ = node["input_width"] ? node["input_width"].as<int>() : 224;
        input_height_ = node["input_height"] ? node["input_height"].as<int>() : 224;
        hsv_min_color_ratio_ = node["hsv_min_color_ratio"] ? node["hsv_min_color_ratio"].as<float>() : hsv_min_color_ratio_;
        hsv_min_dominance_ratio_ =
            node["hsv_min_dominance_ratio"] ? node["hsv_min_dominance_ratio"].as<float>() : hsv_min_dominance_ratio_;
        debug_log_predictions_ =
            node["debug_log_predictions"] ? node["debug_log_predictions"].as<bool>() : debug_log_predictions_;
        debug_top_k_ = node["debug_top_k"] ? std::max(1, node["debug_top_k"].as<int>()) : debug_top_k_;

        if (node["target_labels"]) {
            target_labels_.clear();
            for (const auto &item : node["target_labels"]) {
                target_labels_.insert(NormalizeToken(item.as<std::string>()));
            }
        }
        if (target_labels_.empty()) {
            target_labels_ = {"opponent"};
        }

        if (node["classnames"]) {
            classnames_.clear();
            for (const auto &item : node["classnames"]) {
                classnames_.push_back(NormalizeToken(item.as<std::string>()));
            }
        }

        // Neutral jerseys are harder than saturated ones, so they get slightly
        // lower defaults before we ask the HSV sanity-check to confirm them.
        class_confidence_thresholds_ = {
            {"white", 0.35f},
            {"black", 0.40f},
            {"gray", 0.45f},
            {"brown", 0.45f},
            {"yellow", 0.50f},
            {"orange", 0.50f},
        };
        if (node["confidence_thresholds"]) {
            for (const auto &item : node["confidence_thresholds"]) {
                class_confidence_thresholds_[NormalizeToken(item.first.as<std::string>())] = item.second.as<float>();
            }
        }

        if (node["crop"]) {
            const auto crop = node["crop"];
            use_torso_crop_ = crop["use_torso_crop"] ? crop["use_torso_crop"].as<bool>() : true;
            torso_x0_ = crop["x0"] ? crop["x0"].as<float>() : torso_x0_;
            torso_x1_ = crop["x1"] ? crop["x1"].as<float>() : torso_x1_;
            torso_y0_ = crop["y0"] ? crop["y0"].as<float>() : torso_y0_;
            torso_y1_ = crop["y1"] ? crop["y1"].as<float>() : torso_y1_;
        }

        if (node["stats_crop"]) {
            const auto stats_crop = node["stats_crop"];
            stats_x0_ = stats_crop["x0"] ? stats_crop["x0"].as<float>() : stats_x0_;
            stats_x1_ = stats_crop["x1"] ? stats_crop["x1"].as<float>() : stats_x1_;
            stats_y0_ = stats_crop["y0"] ? stats_crop["y0"].as<float>() : stats_y0_;
            stats_y1_ = stats_crop["y1"] ? stats_crop["y1"].as<float>() : stats_y1_;
        }

        LoadHSVBounds(node);

        model_path_ = node["model_path"] ? node["model_path"].as<std::string>() : "";
        ready_ = false;

        if (!enabled_) {
            std::cout << "robot color classifier disabled by config" << std::endl;
            return;
        }

        if (!model_path_.empty()) {
            try {
                if (!std::filesystem::exists(model_path_)) {
                    std::cerr << "robot color classifier model not found: " << model_path_ << std::endl;
                } else {
                    net_ = cv::dnn::readNet(model_path_);
                    if (!net_.empty()) {
                        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                        ready_ = true;
                        std::cout << "loaded robot color classifier model: " << model_path_ << std::endl;
                    }
                }
            } catch (const std::exception &e) {
                std::cerr << "failed to load robot color classifier model: " << e.what() << std::endl;
            }
        }

        if (!ready_ && hsv_enabled_) {
            std::cout << "robot color classifier using HSV fallback" << std::endl;
        }
    }

    bool Enabled() const {
        return enabled_;
    }

    bool IsReady() const {
        return enabled_ && (ready_ || hsv_enabled_);
    }

    bool ShouldClassify(const std::string &detection_label) const {
        if (!enabled_) {
            return false;
        }
        return target_labels_.find(NormalizeToken(detection_label)) != target_labels_.end();
    }

    cv::Mat ExtractCrop(const cv::Mat &image, const cv::Rect &bbox) const {
        if (image.empty()) {
            return cv::Mat();
        }

        const cv::Rect image_bounds(0, 0, image.cols, image.rows);
        cv::Rect safe_bbox = bbox & image_bounds;
        if (safe_bbox.width <= 1 || safe_bbox.height <= 1) {
            return cv::Mat();
        }

        if (!use_torso_crop_) {
            return image(safe_bbox).clone();
        }

        const int x0 = safe_bbox.x + static_cast<int>(std::round(safe_bbox.width * torso_x0_));
        const int x1 = safe_bbox.x + static_cast<int>(std::round(safe_bbox.width * torso_x1_));
        const int y0 = safe_bbox.y + static_cast<int>(std::round(safe_bbox.height * torso_y0_));
        const int y1 = safe_bbox.y + static_cast<int>(std::round(safe_bbox.height * torso_y1_));

        cv::Rect torso_box(
            std::clamp(std::min(x0, x1), image_bounds.x, image_bounds.width - 1),
            std::clamp(std::min(y0, y1), image_bounds.y, image_bounds.height - 1),
            std::max(1, std::abs(x1 - x0)),
            std::max(1, std::abs(y1 - y0)));
        torso_box &= image_bounds;

        if (torso_box.width <= 1 || torso_box.height <= 1) {
            return image(safe_bbox).clone();
        }
        return image(torso_box).clone();
    }

    Prediction Predict(const cv::Mat &input_img) {
        if (!enabled_ || input_img.empty()) {
            return {};
        }

        if (ready_) {
            auto prediction = PredictWithModel(input_img);
            if (prediction.valid) {
                return prediction;
            }

            // If the model is close but not confident enough, confirm the same
            // color inside a smaller center patch before giving up on it.
            prediction = RescueWeakModelPrediction(prediction, input_img);
            if (prediction.valid) {
                return prediction;
            }
        }

        if (hsv_enabled_) {
            return PredictWithHSV(input_img);
        }
        return {};
    }

    std::string Classify(const cv::Mat &input_img) {
        // The rest of the pipeline expects a concrete string. Keep empty
        // predictions internal and expose "unknown" at the API boundary.
        const auto prediction = Predict(input_img);
        return prediction.valid && !prediction.label.empty() ? prediction.label : unknown_label_;
    }

private:
    static std::string NormalizeToken(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        if (value == "grey") {
            return "gray";
        }
        return value;
    }

    void LoadHSVBounds(const YAML::Node &node) {
        LoadBoundsForKey(node, "red_bounds", red_bounds_);
        LoadBoundsForKey(node, "blue_bounds", blue_bounds_);
        LoadBoundsForKey(node, "yellow_bounds", yellow_bounds_);
        LoadBoundsForKey(node, "orange_bounds", orange_bounds_);
        LoadBoundsForKey(node, "purple_bounds", purple_bounds_);
        LoadBoundsForKey(node, "white_bounds", white_bounds_);
        LoadBoundsForKey(node, "black_bounds", black_bounds_);
        LoadBoundsForKey(node, "gray_bounds", gray_bounds_);
        LoadBoundsForKey(node, "brown_bounds", brown_bounds_);
        LoadBoundsForKey(node, "green_bounds", green_bounds_);

        // Direct HSV fallback can still consider neutrals, but they pass
        // stricter checks than vivid jerseys so shirtless robots stay unknown.
        hsv_enabled_ =
            !red_bounds_.empty() ||
            !blue_bounds_.empty() ||
            !yellow_bounds_.empty() ||
            !orange_bounds_.empty() ||
            !purple_bounds_.empty() ||
            !white_bounds_.empty() ||
            !black_bounds_.empty() ||
            !gray_bounds_.empty() ||
            !brown_bounds_.empty() ||
            !green_bounds_.empty();
    }

    Prediction PredictWithModel(const cv::Mat &input_img) {
        Prediction prediction;
        if (net_.empty()) {
            return prediction;
        }

        cv::Mat blob = PrepareModelInputBlob(input_img);
        if (blob.empty()) {
            return prediction;
        }

        net_.setInput(blob);
        cv::Mat logits = net_.forward();
        if (logits.empty()) {
            return prediction;
        }

        cv::Mat flat = logits.reshape(1, 1);
        if (flat.total() == 0) {
            return prediction;
        }

        double max_logit = -DBL_MAX;
        for (int idx = 0; idx < flat.cols; ++idx) {
            max_logit = std::max(max_logit, static_cast<double>(flat.at<float>(0, idx)));
        }

        double exp_sum = 0.0;
        std::vector<double> probs(flat.cols, 0.0);
        for (int idx = 0; idx < flat.cols; ++idx) {
            probs[idx] = std::exp(static_cast<double>(flat.at<float>(0, idx)) - max_logit);
            exp_sum += probs[idx];
        }

        if (exp_sum <= 0.0) {
            return prediction;
        }

        int best_idx = -1;
        double best_prob = 0.0;
        for (int idx = 0; idx < flat.cols; ++idx) {
            probs[idx] /= exp_sum;
            if (probs[idx] > best_prob) {
                best_prob = probs[idx];
                best_idx = idx;
            }
        }

        if (best_idx < 0) {
            return prediction;
        }

        prediction.label = best_idx < static_cast<int>(classnames_.size()) ? classnames_[best_idx] : std::to_string(best_idx);
        prediction.confidence = static_cast<float>(best_prob);

        const float required_threshold = GetRequiredThreshold(prediction.label);
        if (prediction.confidence < required_threshold) {
            if (debug_log_predictions_) {
                std::cout << "robot color classifier rejected crop: "
                          << FormatTopPredictions(probs)
                          << " threshold(" << prediction.label << ")=" << std::fixed << std::setprecision(3)
                          << required_threshold << std::endl;
            }
            return prediction;
        }

        prediction.valid = true;
        if (debug_log_predictions_) {
            std::cout << "robot color classifier accepted crop: "
                      << FormatTopPredictions(probs) << std::endl;
        }
        return prediction;
    }

    cv::Mat ExtractStatsCrop(const cv::Mat &input_img) const {
        if (input_img.empty()) {
            return cv::Mat();
        }

        const cv::Rect image_bounds(0, 0, input_img.cols, input_img.rows);
        const int x0 = static_cast<int>(std::round(input_img.cols * stats_x0_));
        const int x1 = static_cast<int>(std::round(input_img.cols * stats_x1_));
        const int y0 = static_cast<int>(std::round(input_img.rows * stats_y0_));
        const int y1 = static_cast<int>(std::round(input_img.rows * stats_y1_));

        cv::Rect stats_box(
            std::clamp(std::min(x0, x1), image_bounds.x, image_bounds.width - 1),
            std::clamp(std::min(y0, y1), image_bounds.y, image_bounds.height - 1),
            std::max(1, std::abs(x1 - x0)),
            std::max(1, std::abs(y1 - y0)));
        stats_box &= image_bounds;

        if (stats_box.width <= 1 || stats_box.height <= 1) {
            return input_img.clone();
        }
        return input_img(stats_box).clone();
    }

    cv::Mat BuildHSVImage(const cv::Mat &input_img) const {
        if (input_img.empty()) {
            return cv::Mat();
        }

        cv::Mat hsv;
        cv::cvtColor(input_img, hsv, cv::COLOR_BGR2HSV);
        cv::GaussianBlur(hsv, hsv, cv::Size(5, 5), 0);
        return hsv;
    }

    cv::Mat GetColorMask(
        const cv::Mat &input_img,
        const std::vector<std::pair<cv::Scalar, cv::Scalar>> &color_bounds) const {
        cv::Mat mask = cv::Mat::zeros(input_img.size(), CV_8UC1);
        for (const auto &bounds : color_bounds) {
            cv::Mat mask_tmp;
            cv::inRange(input_img, bounds.first, bounds.second, mask_tmp);
            cv::bitwise_or(mask, mask_tmp, mask);
        }
        return mask;
    }

    float ComputeColorRatio(const cv::Mat &hsv, const std::string &label) const {
        const auto &bounds = GetBoundsForLabel(label);
        if (hsv.empty() || bounds.empty() || hsv.total() == 0) {
            return 0.0f;
        }
        return static_cast<float>(cv::countNonZero(GetColorMask(hsv, bounds))) / static_cast<float>(hsv.total());
    }

    Prediction PredictWithHSV(const cv::Mat &input_img) const {
        Prediction prediction;

        cv::Mat stats_crop = ExtractStatsCrop(input_img);
        cv::Mat hsv = BuildHSVImage(stats_crop);
        if (hsv.empty()) {
            return prediction;
        }

        std::vector<ColorScore> scores;
        scores.reserve(10);
        AddColorScore(hsv, "red", red_bounds_, scores);
        AddColorScore(hsv, "blue", blue_bounds_, scores);
        AddColorScore(hsv, "yellow", yellow_bounds_, scores);
        AddColorScore(hsv, "orange", orange_bounds_, scores);
        AddColorScore(hsv, "purple", purple_bounds_, scores);
        AddColorScore(hsv, "white", white_bounds_, scores);
        AddColorScore(hsv, "black", black_bounds_, scores);
        AddColorScore(hsv, "gray", gray_bounds_, scores);
        AddColorScore(hsv, "brown", brown_bounds_, scores);
        AddColorScore(hsv, "green", green_bounds_, scores);

        if (scores.empty()) {
            return prediction;
        }
        std::sort(scores.begin(), scores.end(), [](const ColorScore &lhs, const ColorScore &rhs) {
            return lhs.count > rhs.count;
        });

        const int dominant_count = scores[0].count;
        if (dominant_count <= 0 || hsv.total() == 0) {
            return prediction;
        }

        const int runner_up_count = scores.size() > 1 ? scores[1].count : 0;
        const float dominant_ratio = static_cast<float>(dominant_count) / static_cast<float>(hsv.total());

        if (dominant_ratio < GetRequiredColorRatio(scores[0].label)) {
            return prediction;
        }

        if (runner_up_count > 0 &&
            static_cast<float>(dominant_count) <
                static_cast<float>(runner_up_count) * GetRequiredDominance(scores[0].label)) {
            return prediction;
        }

        int sum_other_counts = 0;
        for (size_t idx = 1; idx < scores.size(); ++idx) {
            sum_other_counts += scores[idx].count;
        }

        if (scores[0].label == "green" && dominant_count <= (sum_other_counts * 3)) {
            return prediction;
        }

        if (!PassesLabelSpecificColorCheck(hsv, scores[0].label, dominant_count)) {
            return prediction;
        }

        prediction.label = scores[0].label;
        prediction.confidence = dominant_ratio;
        prediction.valid = true;
        return prediction;
    }

    Prediction RescueWeakModelPrediction(const Prediction &model_prediction, const cv::Mat &input_img) const {
        Prediction prediction;
        if (model_prediction.label.empty()) {
            return prediction;
        }

        const std::string candidate_label = NormalizeToken(model_prediction.label);
        if (IsNeutralLabel(candidate_label)) {
            return RescueNeutralPrediction(model_prediction, input_img);
        }

        const auto &bounds = GetBoundsForLabel(candidate_label);
        if (bounds.empty()) {
            return prediction;
        }

        if (model_prediction.confidence < GetRescueConfidenceThreshold(candidate_label)) {
            return prediction;
        }

        cv::Mat stats_crop = ExtractStatsCrop(input_img);
        cv::Mat hsv = BuildHSVImage(stats_crop);
        if (hsv.empty()) {
            return prediction;
        }

        const float candidate_ratio = ComputeColorRatio(hsv, candidate_label);
        if (candidate_ratio < GetRescueColorRatioThreshold(candidate_label)) {
            return prediction;
        }

        const int candidate_count = static_cast<int>(std::round(candidate_ratio * static_cast<float>(hsv.total())));
        const int runner_up_count = GetStrongestCompetingColorCount(hsv, candidate_label);
        if (runner_up_count > 0 &&
            static_cast<float>(candidate_count) <
                static_cast<float>(runner_up_count) * GetRescueDominanceThreshold(candidate_label)) {
            return prediction;
        }

        prediction.label = candidate_label;
        prediction.confidence = std::max(model_prediction.confidence, candidate_ratio);
        prediction.valid = true;

        if (debug_log_predictions_) {
            std::cout << "robot color classifier rescued crop as " << candidate_label
                      << " model=" << std::fixed << std::setprecision(3) << model_prediction.confidence
                      << " hsv=" << candidate_ratio << std::endl;
        }
        return prediction;
    }

    Prediction RescueNeutralPrediction(const Prediction &model_prediction, const cv::Mat &input_img) const {
        Prediction prediction;
        if (model_prediction.confidence < 0.12f) {
            return prediction;
        }

        cv::Mat stats_crop = ExtractStatsCrop(input_img);
        cv::Mat hsv = BuildHSVImage(stats_crop);
        if (hsv.empty()) {
            return prediction;
        }

        std::vector<ColorScore> neutral_scores;
        neutral_scores.reserve(4);
        AddColorScore(hsv, "black", black_bounds_, neutral_scores);
        AddColorScore(hsv, "gray", gray_bounds_, neutral_scores);
        AddColorScore(hsv, "white", white_bounds_, neutral_scores);
        AddColorScore(hsv, "brown", brown_bounds_, neutral_scores);
        if (neutral_scores.empty()) {
            return prediction;
        }

        std::sort(neutral_scores.begin(), neutral_scores.end(), [](const ColorScore &lhs, const ColorScore &rhs) {
            return lhs.count > rhs.count;
        });

        const auto &best_neutral = neutral_scores.front();
        if (best_neutral.count <= 0 || hsv.total() == 0) {
            return prediction;
        }

        const int runner_up_count = neutral_scores.size() > 1 ? neutral_scores[1].count : 0;
        const float candidate_ratio = static_cast<float>(best_neutral.count) / static_cast<float>(hsv.total());

        if (candidate_ratio < GetRescueColorRatioThreshold(best_neutral.label)) {
            return prediction;
        }

        if (runner_up_count > 0 &&
            static_cast<float>(best_neutral.count) <
                static_cast<float>(runner_up_count) * GetRescueDominanceThreshold(best_neutral.label)) {
            return prediction;
        }

        if (!PassesLabelSpecificColorCheck(hsv, best_neutral.label, best_neutral.count)) {
            return prediction;
        }

        prediction.label = best_neutral.label;
        prediction.confidence = std::max(model_prediction.confidence, candidate_ratio);
        prediction.valid = true;

        if (debug_log_predictions_) {
            std::cout << "robot color classifier rescued neutral crop as " << best_neutral.label
                      << " model=" << std::fixed << std::setprecision(3) << model_prediction.confidence
                      << " hsv=" << candidate_ratio << std::endl;
        }
        return prediction;
    }

    cv::Mat PrepareModelInputBlob(const cv::Mat &input_img) const {
        if (input_img.empty() || input_width_ <= 0 || input_height_ <= 0) {
            return cv::Mat();
        }

        cv::Mat resized;
        if (input_width_ == input_height_) {
            const int short_edge = std::min(input_img.cols, input_img.rows);
            if (short_edge <= 0) {
                return cv::Mat();
            }
            const float scale = static_cast<float>(input_width_) / static_cast<float>(short_edge);
            const int resized_width = std::max(input_width_, static_cast<int>(std::round(input_img.cols * scale)));
            const int resized_height = std::max(input_height_, static_cast<int>(std::round(input_img.rows * scale)));
            cv::resize(input_img, resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_LINEAR);
        } else {
            cv::resize(input_img, resized, cv::Size(input_width_, input_height_), 0.0, 0.0, cv::INTER_LINEAR);
        }

        if (resized.empty()) {
            return cv::Mat();
        }

        const int crop_x = std::max(0, (resized.cols - input_width_) / 2);
        const int crop_y = std::max(0, (resized.rows - input_height_) / 2);
        const int crop_w = std::min(input_width_, resized.cols);
        const int crop_h = std::min(input_height_, resized.rows);
        cv::Rect crop_box(crop_x, crop_y, crop_w, crop_h);
        cv::Mat cropped = resized(crop_box).clone();
        if (cropped.empty()) {
            return cv::Mat();
        }

        if (cropped.cols != input_width_ || cropped.rows != input_height_) {
            cv::copyMakeBorder(
                cropped,
                cropped,
                0,
                std::max(0, input_height_ - cropped.rows),
                0,
                std::max(0, input_width_ - cropped.cols),
                cv::BORDER_REPLICATE);
        }

        cv::Mat rgb;
        cv::cvtColor(cropped, rgb, cv::COLOR_BGR2RGB);
        rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

        std::vector<cv::Mat> channels;
        cv::split(rgb, channels);
        for (size_t idx = 0; idx < channels.size() && idx < model_mean_.size() && idx < model_std_.size(); ++idx) {
            channels[idx] = (channels[idx] - model_mean_[idx]) / model_std_[idx];
        }
        cv::merge(channels, rgb);

        return cv::dnn::blobFromImage(
            rgb,
            1.0,
            cv::Size(input_width_, input_height_),
            cv::Scalar(),
            false,
            false,
            CV_32F);
    }

    float GetRequiredThreshold(const std::string &label) const {
        const auto it = class_confidence_thresholds_.find(label);
        return it != class_confidence_thresholds_.end() ? it->second : confidence_threshold_;
    }

    float GetRequiredColorRatio(const std::string &label) const {
        if (label == "white") {
            return std::max(hsv_min_color_ratio_, 0.12f);
        }
        if (label == "black" || label == "gray") {
            return std::max(hsv_min_color_ratio_, 0.10f);
        }
        if (label == "brown") {
            return std::max(hsv_min_color_ratio_, 0.09f);
        }
        return hsv_min_color_ratio_;
    }

    float GetRequiredDominance(const std::string &label) const {
        if (label == "white") {
            return std::max(hsv_min_dominance_ratio_, 1.50f);
        }
        if (label == "black" || label == "gray") {
            return std::max(hsv_min_dominance_ratio_, 1.10f);
        }
        if (label == "brown") {
            return std::max(hsv_min_dominance_ratio_, 1.05f);
        }
        return hsv_min_dominance_ratio_;
    }

    float GetRescueConfidenceThreshold(const std::string &label) const {
        const float default_threshold = GetRequiredThreshold(label);
        if (IsNeutralLabel(label)) {
            return std::max(0.12f, default_threshold - 0.20f);
        }
        return std::max(0.20f, default_threshold - 0.10f);
    }

    float GetRescueColorRatioThreshold(const std::string &label) const {
        if (label == "white") {
            return 0.10f;
        }
        if (label == "black" || label == "gray") {
            return 0.08f;
        }
        if (label == "brown") {
            return 0.06f;
        }
        return GetRequiredColorRatio(label);
    }

    float GetRescueDominanceThreshold(const std::string &label) const {
        if (IsNeutralLabel(label)) {
            return 1.10f;
        }
        return GetRequiredDominance(label);
    }

    bool IsNeutralLabel(const std::string &label) const {
        return label == "white" || label == "black" || label == "gray" || label == "brown";
    }

    bool PassesLabelSpecificColorCheck(const cv::Mat &hsv, const std::string &label, int dominant_count) const {
        if (dominant_count <= 0) {
            return false;
        }

        const int black_count = CountColorPixels(hsv, "black");
        const int gray_count = CountColorPixels(hsv, "gray");
        const int white_count = CountColorPixels(hsv, "white");
        const int brown_count = CountColorPixels(hsv, "brown");

        if (label == "black") {
            // A black jersey can include white numbers, but black should still
            // meaningfully beat the white/gray torso content in the center.
            return static_cast<float>(dominant_count) >= static_cast<float>(white_count + gray_count) * 0.85f;
        }
        if (label == "white") {
            return static_cast<float>(dominant_count) >= static_cast<float>(black_count) * 0.60f;
        }
        if (label == "gray") {
            return static_cast<float>(dominant_count) >= static_cast<float>(black_count + white_count) * 0.70f;
        }
        if (label == "brown") {
            return static_cast<float>(dominant_count) >= static_cast<float>(black_count + gray_count) * 0.70f;
        }
        return true;
    }

    int GetStrongestCompetingColorCount(const cv::Mat &hsv, const std::string &skip_label) const {
        int strongest = 0;
        for (const std::string &label : kAllColorLabels_) {
            if (label == skip_label) {
                continue;
            }
            strongest = std::max(strongest, CountColorPixels(hsv, label));
        }
        return strongest;
    }

    int CountColorPixels(const cv::Mat &hsv, const std::string &label) const {
        const auto &bounds = GetBoundsForLabel(label);
        if (hsv.empty() || bounds.empty()) {
            return 0;
        }
        return cv::countNonZero(GetColorMask(hsv, bounds));
    }

    const std::vector<std::pair<cv::Scalar, cv::Scalar>> &GetBoundsForLabel(const std::string &label) const {
        if (label == "red") {
            return red_bounds_;
        }
        if (label == "blue") {
            return blue_bounds_;
        }
        if (label == "yellow") {
            return yellow_bounds_;
        }
        if (label == "orange") {
            return orange_bounds_;
        }
        if (label == "purple") {
            return purple_bounds_;
        }
        if (label == "white") {
            return white_bounds_;
        }
        if (label == "black") {
            return black_bounds_;
        }
        if (label == "gray") {
            return gray_bounds_;
        }
        if (label == "brown") {
            return brown_bounds_;
        }
        if (label == "green") {
            return green_bounds_;
        }
        return empty_bounds_;
    }

    std::string FormatTopPredictions(const std::vector<double> &probs) const {
        std::vector<int> order(probs.size());
        for (int idx = 0; idx < static_cast<int>(probs.size()); ++idx) {
            order[idx] = idx;
        }
        std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
            return probs[lhs] > probs[rhs];
        });

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3) << "top" << debug_top_k_ << ": ";
        const int limit = std::min<int>(debug_top_k_, static_cast<int>(order.size()));
        for (int idx = 0; idx < limit; ++idx) {
            if (idx > 0) {
                oss << " ";
            }
            const int class_idx = order[idx];
            const std::string label =
                class_idx < static_cast<int>(classnames_.size()) ? classnames_[class_idx] : std::to_string(class_idx);
            oss << label << "=" << probs[class_idx];
        }
        return oss.str();
    }

    bool enabled_ = true;
    bool ready_ = false;
    bool hsv_enabled_ = false;
    bool use_torso_crop_ = true;

    float confidence_threshold_ = 0.60f;
    float hsv_min_color_ratio_ = 0.08f;
    float hsv_min_dominance_ratio_ = 1.35f;
    float torso_x0_ = 0.20f;
    float torso_x1_ = 0.80f;
    float torso_y0_ = 0.15f;
    float torso_y1_ = 0.65f;
    // Use a tighter inner patch for rescue/fallback so limbs and turf do not
    // dominate the jersey signal.
    float stats_x0_ = 0.18f;
    float stats_x1_ = 0.82f;
    float stats_y0_ = 0.12f;
    float stats_y1_ = 0.72f;
    int input_width_ = 224;
    int input_height_ = 224;
    int debug_top_k_ = 3;
    bool debug_log_predictions_ = false;
    std::array<float, 3> model_mean_ = {0.485f, 0.456f, 0.406f};
    std::array<float, 3> model_std_ = {0.229f, 0.224f, 0.225f};

    const std::string unknown_label_ = "unknown";
    const std::vector<std::string> kAllColorLabels_ = {
        "red", "blue", "yellow", "orange", "purple", "white", "black", "gray", "brown", "green"};

    std::string model_path_;
    std::unordered_set<std::string> target_labels_;
    std::unordered_map<std::string, float> class_confidence_thresholds_;
    std::vector<std::string> classnames_;
    cv::dnn::Net net_;

    struct ColorScore {
        std::string label;
        int count;
    };

    static void LoadBoundsForKey(
        const YAML::Node &node,
        const char *key,
        std::vector<std::pair<cv::Scalar, cv::Scalar>> &target_bounds) {
        if (!node[key]) {
            return;
        }
        target_bounds.clear();
        for (const auto &bound : node[key]) {
            if (bound.size() != 6) {
                continue;
            }
            target_bounds.emplace_back(
                cv::Scalar(bound[0].as<int>(), bound[1].as<int>(), bound[2].as<int>()),
                cv::Scalar(bound[3].as<int>(), bound[4].as<int>(), bound[5].as<int>()));
        }
    }

    void AddColorScore(
        const cv::Mat &hsv,
        const std::string &label,
        const std::vector<std::pair<cv::Scalar, cv::Scalar>> &bounds,
        std::vector<ColorScore> &scores) const {
        if (bounds.empty()) {
            return;
        }
        scores.push_back(ColorScore{label, cv::countNonZero(GetColorMask(hsv, bounds))});
    }

    const std::vector<std::pair<cv::Scalar, cv::Scalar>> empty_bounds_;
    std::vector<std::pair<cv::Scalar, cv::Scalar>> red_bounds_ = {
        {cv::Scalar(0, 90, 50), cv::Scalar(12, 255, 255)},
        {cv::Scalar(165, 90, 80), cv::Scalar(179, 255, 255)}};
    std::vector<std::pair<cv::Scalar, cv::Scalar>> blue_bounds_ = {
        {cv::Scalar(95, 120, 50), cv::Scalar(140, 255, 255)}};
    std::vector<std::pair<cv::Scalar, cv::Scalar>> yellow_bounds_ = {
        {cv::Scalar(20, 90, 90), cv::Scalar(38, 255, 255)}};
    std::vector<std::pair<cv::Scalar, cv::Scalar>> orange_bounds_ = {
        {cv::Scalar(8, 120, 80), cv::Scalar(22, 255, 255)}};
    std::vector<std::pair<cv::Scalar, cv::Scalar>> purple_bounds_ = {
        {cv::Scalar(125, 70, 50), cv::Scalar(160, 255, 255)}};
    std::vector<std::pair<cv::Scalar, cv::Scalar>> white_bounds_ = {
        {cv::Scalar(0, 0, 170), cv::Scalar(179, 55, 255)}};
    std::vector<std::pair<cv::Scalar, cv::Scalar>> black_bounds_ = {
        {cv::Scalar(0, 0, 0), cv::Scalar(179, 255, 60)}};
    std::vector<std::pair<cv::Scalar, cv::Scalar>> gray_bounds_ = {
        {cv::Scalar(0, 0, 50), cv::Scalar(179, 55, 170)}};
    std::vector<std::pair<cv::Scalar, cv::Scalar>> brown_bounds_ = {
        {cv::Scalar(5, 70, 25), cv::Scalar(25, 220, 165)}};
    std::vector<std::pair<cv::Scalar, cv::Scalar>> green_bounds_ = {
        {cv::Scalar(30, 45, 45), cv::Scalar(80, 255, 255)}};
};

} // namespace booster_vision
