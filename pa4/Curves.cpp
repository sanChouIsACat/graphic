//#include "Curves.hpp"
//#include "logger.hpp"
//#include "algebra.hpp"
//
// using namespace g_curves::curves_internal;
// const cv::Scalar WHITE{ 255,255,255 };
// const cv::Scalar BLACK{ 0,0,0 };
// void g_curves::Curves::draw()
//{
//    cv::imshow(replica_window_name, main_window);
//    cv::imshow(main_window_name, replica_window);
//    cv::setMouseCallback(main_window_name, mouse_handler, this);
//
//
//    while (true) {
//        bool exit = false;
//        if (event_loop.empty()) {
//            //std::this_thread::sleep_for(std::chrono::milliseconds(50));
//        }
//        else {
//            const EventFunctionPack& event = event_loop.front();
//            //G_LOGGER_INFO("event %s triggered for main",
//            event.name.c_str()); exit = event.function(main_window);
//            //G_LOGGER_INFO("event %s triggered for replica",
//            event.name.c_str()); event.function(replica_window);
//            event_loop.pop();
//        }
//        cv::imshow(replica_window_name, main_window);
//        cv::imshow(main_window_name, replica_window);
//        int key = cv::waitKey(1);
//        if (exit || key == 27) {
//            break;
//        }
//    }
//}
// bool drawTemporaryControlSeq(cv::Mat& window,
//    std::unordered_set<POINT_EGDE_2D>::iterator begin,
//    std::unordered_set<POINT_EGDE_2D>::iterator end,
//    std::unordered_set<POINT_EGDE_2D>::iterator originSeq,
//    cv::Point temporary_point,
//    float radius,
//    const cv::Scalar& color) {
//    auto draw = [&](const cv::Point& point) {
//        cv::circle(window, point, radius, color, -1);
//        };
//
//    while (begin != end) {
//        if (begin == originSeq) {
//            continue;
//        }
//        draw(cv::Point{ (*begin).x(),(*begin).y() });
//        begin++;
//    };
//
//    draw(temporary_point);
//    return false;
//}
//
// bool drawControlSeq(cv::Mat& window,
//    std::unordered_set<POINT_EGDE_2D>::iterator begin,
//    std::unordered_set<POINT_EGDE_2D>::iterator end,
//    float radius,
//    const cv::Scalar& color) {
//    auto draw = [&](const cv::Point& point) {
//        cv::circle(window, point, radius, color, -1);
//        };
//
//    while (begin != end) {
//        draw(cv::Point{ (*begin).x(),(*begin).y() });
//        begin++;
//    };
//
//    return false;
//}
//
// bool moveCircle(cv::Mat& window, cv::Point old_center, cv::Point new_center,
// float radius, const cv::Scalar& color) {
//    cv::circle(window, old_center, radius, BLACK, -1);
//    cv::circle(window, new_center, radius, color, -1);
//    return false;
//}
//
// int tick = 0;
// int sample_feq = 5;
//
// void g_curves::Curves::mouse_handler(int event, int x, int y, int flags,
// void* userdata)
//{
//    tick = (tick + 1) % sample_feq;
//    Curves* _this = static_cast<Curves*>(userdata);
//    POINT_EGDE_2D coor{ (float)x,(float)y,1 };
//    auto circleFinder = [&coor, radiu = _this->control_circle_radiu](const
//    POINT_EGDE_2D& a) {
//        return algebra::insideCircle(a, coor, radiu);
//        };
//    // just for debugging
//    if (event != cv::EVENT_MOUSEMOVE) {
//        G_LOGGER_INFO("event [%d] trigged ", event);
//    }
//
//    // using if statement at monent, consider replace with state machine.
//    if (event == cv::EVENT_LBUTTONDOWN) {
//        _this->mouse_state = down;
//        std::unordered_set<POINT_EGDE_2D>::iterator find_it =
//        std::find_if(_this->control_points.begin(),
//            _this->control_points.end(),
//            circleFinder
//        );
//
//        if (find_it == _this->control_points.end()) {
//            return;
//        }
//
//        _this->inside_circle = find_it != _this->control_points.end();
//        _this->last_x = x;
//        _this->last_y = y;
//        // check if point
//
//        // cache it for direct using in sending EVENT_MOUSEMOVE
//        _this->iterator_cache = find_it;
//        G_LOGGER_INFO("mouse down , inside trigger %d", _this->inside_circle);
//    }
//    // move circle
//    else if (event == cv::EVENT_MOUSEMOVE &&
//        _this->mouse_state == down &&
//        _this->inside_circle &&
//        (_this->last_x != x || _this->last_y != y)) {
//
//        // recude sample rate
//        if (tick) {
//            return ;
//        }
//        const POINT_EGDE_2D& old_center = *_this->iterator_cache;
//        // the compiler can't auto deduce parameters. Wonder why?
//        EventFunctionPack packed = packEvent<cv::Point,cv::Point, float, const
//        cv::Scalar&>(moveCircle,
//            "move circle",
//            cv::Point{ (int)old_center.x(), (int)old_center.y()},
//            cv::Point{ x,y },
//            _this->control_circle_radiu,
//            WHITE);
//        _this->control_points.erase(_this->iterator_cache);
//        auto [iterator, _] = _this->control_points.emplace(coor);
//        _this->iterator_cache = iterator;
//        _this->event_loop.push(packed);
//    }
//    else if (event == cv::EVENT_LBUTTONUP) {
//        _this->mouse_state = up;
//        if (!_this->inside_circle) {
//            _this->control_points.emplace(coor);
//            _this->event_loop.push(packEvent<cv::Point, float, const
//            cv::Scalar&>(drawTemporaryControlSeq,
//                "draw circle",
//                cv::Point{ x,y },
//                _this->control_circle_radiu,
//                WHITE)
//            );
//        }
//        G_LOGGER_INFO("mouse up");
//    }
//    else if (event == cv::EVENT_RBUTTONUP) {
//        std::unordered_set<POINT_EGDE_2D>::iterator find_it =
//        std::find_if(_this->control_points.begin(),
//            _this->control_points.end(),
//            circleFinder
//        );
//        if (find_it != _this->control_points.end()) {
//            const POINT_EGDE_2D& old = *find_it;
//            _this->event_loop.push(packEvent<cv::Point, float, const
//            cv::Scalar&>(drawTemporaryControlSeq,
//                "erase circle",
//                cv::Point{ (int)old.x(), (int)old.y()},
//                _this->control_circle_radiu,
//                BLACK)
//            );
//            _this->control_points.erase(find_it);
//            G_LOGGER_INFO("erase circle");
//        }
//    }
//}
