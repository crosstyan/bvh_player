/**
***  BVH動作ファイルの読み込み・描画クラス
***  Copyright (c) 2004-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/


#ifndef _BVH_H_
#define _BVH_H_


#include <span>
#include <vector>
#include <map>
#include <string>

//
//  BVH形式のモーションデータ
//
class BVH {
public:
	/*  内部用構造体  */

	// チャンネルの種類
	enum ChannelEnum {
		X_ROTATION,
		Y_ROTATION,
		Z_ROTATION,
		X_POSITION,
		Y_POSITION,
		Z_POSITION
	};

	static const char *Stringify(ChannelEnum e) {
		switch (e) {
		case X_ROTATION:
			return "X Rotation";
		case Y_ROTATION:
			return "Y Rotation";
		case Z_ROTATION:
			return "Z Rotation";
		case X_POSITION:
			return "X Position";
		case Y_POSITION:
			return "Y Position";
		case Z_POSITION:
			return "Z Position";
		}
		return "Unknown";
	}
	static const char *ShortStringify(ChannelEnum e) {
		switch (e) {
		case X_ROTATION:
			return "XR";
		case Y_ROTATION:
			return "YR";
		case Z_ROTATION:
			return "ZR";
		case X_POSITION:
			return "XP";
		case Y_POSITION:
			return "YP";
		case Z_POSITION:
			return "ZP";
		}
		return "UU";
	}

	struct Joint;

	// チャンネル情報
	struct Channel {
		// 対応関節 (reference)
		Joint *joint;

		// チャンネルの種類
		ChannelEnum type;

		// チャンネル番号
		int index;
	};

	// 関節情報
	struct Joint {
		// reference
		BVH *hierarchy;

		// 関節名
		std::string name;
		// 関節番号
		int index;

		// 関節階層（親関節）
		Joint *parent;
		// 関節階層（子関節）
		std::vector<Joint *> children;

		// 接続位置
		double offset[3];

		// 末端位置情報を持つかどうかのフラグ
		bool has_end_site;
		// 末端位置
		double end_site[3];

		// 回転軸
		std::vector<Channel *> channels;

		[[nodiscard]] std::optional<Joint> GetParent() const {
			if (parent) return *parent;
			return std::nullopt;
		}

		[[nodiscard]] std::span<Channel *const> GetChannels() const noexcept { return {channels.cbegin(), channels.cend()}; }
		[[nodiscard]] std::span<const double> GetOffset() const noexcept { return {offset, 3}; }

		[[nodiscard]] std::optional<std::span<const double>> GetEndSite() const {
			if (has_end_site) {
				auto sp = std::span(end_site, 3);
				return std::move(sp);
			}
			return std::nullopt;
		}

		[[nodiscard]] std::optional<double> find_channel(ChannelEnum type) const {
			auto it = std::find_if(channels.cbegin(), channels.cend(), [type](const Channel *c) { return c->type == type; });
			if (it == channels.cend()) return std::nullopt;
			return hierarchy->GetMotionAt(0, (*it)->index);
		}

		/**
		 * @brief get the rotation of the joint at the specified frame
		 * @param frame frame number
		 * @return rotation value (x, y, z) if the channel exists
		 */
		[[nodiscard]] std::optional<std::array<double, 3>> GetRotation(const size_t frame) const {
			if (frame >= hierarchy->GetNumFrame()) return std::nullopt;
			auto e_x_val = find_channel(X_ROTATION);
			if (not e_x_val) return std::nullopt;
			auto x_val   = *e_x_val;
			auto e_y_val = find_channel(Y_ROTATION);
			if (not e_y_val) return std::nullopt;
			auto y_val   = *e_y_val;
			auto e_z_val = find_channel(Z_ROTATION);
			if (not e_z_val) return std::nullopt;
			auto z_val = *e_z_val;
			return std::array{x_val, y_val, z_val};
		}

		/**
		 * @brief get the position of the joint at the specified frame
		 * @param frame frame number
		 * @return position value (x, y, z) if the channel exists
		 */
		[[nodiscard]] std::optional<std::array<double, 3>> GetPosition(const size_t frame) const {
			if (frame >= hierarchy->GetNumFrame()) return std::nullopt;
			auto e_x_val = find_channel(X_POSITION);
			if (not e_x_val) return std::nullopt;
			auto x_val   = *e_x_val;
			auto e_y_val = find_channel(Y_POSITION);
			if (not e_y_val) return std::nullopt;
			auto y_val   = *e_y_val;
			auto e_z_val = find_channel(Z_POSITION);
			if (not e_z_val) return std::nullopt;
			auto z_val = *e_z_val;
			return std::array{x_val, y_val, z_val};
		}
	};


private:
	// ロードが成功したかどうかのフラグ
	bool is_load_success;

	/*  ファイルの情報  */
	std::string file_name;   // ファイル名
	std::string motion_name; // 動作名

	/*  階層構造の情報  */
	int num_channel;                            // チャンネル数
	std::vector<Channel *> channels;            // チャンネル情報 [チャンネル番号] (owning)
	std::vector<Joint *> joints;                // 関節情報 [パーツ番号] (owning)
	std::map<std::string, Joint *> joint_index; // 関節名から関節情報へのインデックス (reference)

	/*  モーションデータの情報  */
	int num_frame;   // フレーム数
	double interval; // フレーム間の時間間隔

	// `f64[*]` with stride of num_channel
	// the shape is `(num_frame, num_channel)`
	//
	// Note that it would be heap allocated in `Load()` method,
	// with length of `num_frame * num_channel`.
	// and would be deallocated in `Clear()` method.
	double *motion = nullptr; // [フレーム番号][チャンネル番号]


public:
	// コンストラクタ・デストラクタ
	BVH(const char *bvh_file_name);
	~BVH();

	// 全情報のクリア
	void Clear();

	// 全情報を設定（関節の階層構造・動作データの設定）
	void Init(const char *name,
			  int n_joi, const Joint **a_joi, int n_chan, const Channel **a_chan,
			  int n_frame, double interval, const double *mo);

	// 関節の階層構造の設定
	void SetSkeleton(const char *name,
					 int n_joi, const Joint **a_joi, int n_chan, const Channel **a_chan);

	// 動作データの設定
	void SetMotion(int n_frame, double interval, const double *mo = nullptr);

	// BVHファイルのロード
	void Load(const char *bvh_file_name);

	// BVHファイルのセーブ
	void Save(const char *bvh_file_name);

public:
	/*  データアクセス関数  */

	[[nodiscard]] std::span<const double> GetMotion() const {
		return {motion, static_cast<size_t>(num_frame * num_channel)};
	}

	/**
	 * @brief  ストライド (チャンネル数)
	 */
	[[nodiscard]] int GetStride() const { return num_channel; }

	// ロードが成功したかどうかを取得
	[[nodiscard]] bool IsLoadSuccess() const { return is_load_success; }

	// ファイルの情報の取得
	[[nodiscard]] const std::string &GetFileName() const { return file_name; }
	[[nodiscard]] const std::string &GetMotionName() const { return motion_name; }

	// 階層構造の情報の取得
	[[nodiscard]] int GetNumJoint() const { return joints.size(); }
	[[nodiscard]] const Joint *GetJoint(int no) const { return joints[no]; }
	[[nodiscard]] int GetNumChannel() const { return channels.size(); }
	[[nodiscard]] const Channel *GetChannel(int no) const { return channels[no]; }

	[[nodiscard]] const Joint *GetJoint(const std::string &j) const {
		auto i = joint_index.find(j);
		return (i != joint_index.end()) ? (*i).second : nullptr;
	}
	const Joint *GetJoint(const char *j) const {
		auto i = joint_index.find(j);
		return (i != joint_index.end()) ? (*i).second : nullptr;
	}

	// モーションデータの情報の取得
	[[nodiscard]] int GetNumFrame() const { return num_frame; }
	[[nodiscard]] double GetInterval() const { return interval; }
	/**
	 * @brief モーションデータの取得
	 * @param f frame
	 * @param c channel
	 * @return motion[f][c]
	 */
	[[nodiscard]] double GetMotionAt(const int f, const int c) const { return motion[f * num_channel + c]; }

	// モーションデータの情報の変更
	void SetMotion(int f, int c, double v) { motion[f * num_channel + c] = v; }

protected:
	/*  セーブの補助関数  */

	// 階層構造を再帰的に出力
	void OutputHierarchy(std::ofstream &file, const Joint *joint, int indent_level,
						 std::vector<int> &channel_list);

public:
	/*  姿勢の描画関数  */

	// 指定フレームの姿勢を描画
	void RenderFigure(int frame_no, float scale = 1.0f);

	// 指定されたBVH骨格・姿勢を描画（クラス関数）
	static void RenderFigure(const Joint *root, const double *data, float scale = 1.0f);

	// BVH骨格の１本のリンクを描画（クラス関数）
	static void RenderBone(float x0, float y0, float z0, float x1, float y1, float z1);
};


#endif // _BVH_H_
