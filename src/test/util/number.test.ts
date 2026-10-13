import * as assert from 'assert';
import { is9wordString } from "../../core/util/number";

suite('Extension Utils Number', () => {
	test('is9wordString length check', () => {
		const length6Words = "aaaaaa";
		const length8Words = "aaaaaaaa";
		const length7Words = "aaaaaaa";
		const length9Words = "aaaaaaaaa";
		assert.strictEqual(false, is9wordString(length6Words));
		assert.strictEqual(false, is9wordString(length8Words));
		assert.strictEqual(false, is9wordString(length7Words));
		assert.strictEqual(true, is9wordString(length9Words));
    });
	test('is9wordString hex check', () => {
		const notHexAlphabetWords1 = "zaaaaaa";
		const notHexAlphabetWords2 = "z000000";
		const notHexJapaneseWords1 = "あaaaaaa";
		const notHexJapaneseWords2 = "あ000000";
		const hexAlphabetWords = "aaaaaaa";
		const hexNumberWords = "1111111";
		const hexWords = "abcdef9"
		assert.strictEqual(false, is9wordString(notHexAlphabetWords1));
		assert.strictEqual(false, is9wordString(notHexAlphabetWords2));
		assert.strictEqual(false, is9wordString(notHexJapaneseWords1));
		assert.strictEqual(false, is9wordString(notHexJapaneseWords2));
		assert.strictEqual(true, is9wordString(hexAlphabetWords));
		assert.strictEqual(true, is9wordString(hexNumberWords));
		assert.strictEqual(true, is9wordString(hexWords));
	})
});

