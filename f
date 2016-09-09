Binary file ./UseLibs/controller.o matches
./UseLibs/include/controller.h:35:		enum KCPP_MOD{ CRC_MODE, CAC_MODE };
./UseLibs/include/controller.h:38:    void run(const std::string& directory, const std::string& image, int k, KCPP_MOD mod);
./UseLibs/include/controller.h:47:    void runkCPP(int k, ReebGraph& graph, std::list<Edge>& eulerCycle, KCPP_MOD mod);
./UseLibs/include/controller.h:55:				std::list<Edge>& eulerCycle, vector<Point2D>& wayPoints, KCPP_MOD mod);
./UseLibs/.make.log:5:       mod = boost::lexical_cast<Controller::KCPP_MOD>(argv[4]);
./UseLibs/.make.log:13:/usr/include/boost/lexical_cast.hpp: In instantiation of ‘struct boost::detail::deduce_target_char_impl<boost::detail::deduce_character_type_later<Controller::KCPP_MOD> >’:
./UseLibs/.make.log:14:/usr/include/boost/lexical_cast.hpp:415:89:   required from ‘struct boost::detail::deduce_target_char<Controller::KCPP_MOD>’
./UseLibs/.make.log:15:/usr/include/boost/lexical_cast.hpp:674:92:   required from ‘struct boost::detail::lexical_cast_stream_traits<char*, Controller::KCPP_MOD>’
./UseLibs/.make.log:16:/usr/include/boost/lexical_cast.hpp:2343:19:   required from ‘static Target boost::detail::lexical_cast_do_cast<Target, Source>::lexical_cast_impl(const Source&) [with Target = Controller::KCPP_MOD; Source = char*]’
./UseLibs/.make.log:17:/usr/include/boost/lexical_cast.hpp:2523:50:   required from ‘Target boost::lexical_cast(const Source&) [with Target = Controller::KCPP_MOD; Source = char*]’
./UseLibs/src/controller.cpp:24:        int k, KCPP_MOD mod)
./UseLibs/src/controller.cpp:84:void Controller::runkCPP(int k, ReebGraph& graph, std::list<Edge>& eulerCycle, KCPP_MOD mod)
./UseLibs/src/controller.cpp:134: * also it actually doesn't have to recieve KCPP_MOD
./UseLibs/src/controller.cpp:141:                std::list<Edge>& eulerCycle, vector<Point2D>& wayPoints, KCPP_MOD mod)
./UseLibs/src/main.cpp:21:    Controller::KCPP_MODE mod;
./UseLibs/src/main.cpp:56:						mod = boost::lexical_cast<Controller::KCPP_MODE>(argv[4]);
grep: input file ‘./f’ is also the output
