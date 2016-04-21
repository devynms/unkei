/*
 * =====================================================================================
 *
 *       Filename:  MainWrapper.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  04/21/2016 06:32:40 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Joshua Tang
 *   Organization:  
 *
 * =====================================================================================
 */
#ifndef __MAIN_WRAPPER_H
#define __MAIN_WRAPPER_H


#include <iostream>
#include <boost/thread/thread.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

using namespace std;

bool done = false;
boost::unique_lock done_mutex;
boost::condition_variable done_cond;



